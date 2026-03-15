import time
import json
import re

from . import bus
from .afc_mfrc522 import MFRC522Handler


def _parse_lane_list(cfg_value):
    if cfg_value is None:
        return set()
    s = str(cfg_value).strip()
    if not s:
        return set()
    parts = re.split(r"[,\s]+", s)
    out = set()
    for p in parts:
        p = p.strip()
        if not p:
            continue
        if p.lower().startswith("lane"):
            out.add(p.lower())
        elif p.isdigit():
            out.add("lane%s" % p)
        else:
            out.add(p.lower())
    return out


class AfcRfid:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]

        self.spi_speed = int(config.getint("spi_speed", 100000, minval=10000))

        # lane mapping for this reader
        lanes_cfg = None
        try:
            lanes_cfg = config.get("lane", None)
        except Exception:
            lanes_cfg = None
        if lanes_cfg is None:
            try:
                lanes_cfg = config.get("lanes", None)
            except Exception:
                lanes_cfg = None
        self.lanes = _parse_lane_list(lanes_cfg)

        # pending spool assignment per lane (stored on the reader instance that owns that lane)
        # { "lane3": {"spoolman_id":123,"uid_hex":"..","ts":..., "timeout":...} }
        self._pending = {}

        # SPI + MFRC522
        self.spi = bus.MCU_SPI_from_config(config, mode=0, default_speed=self.spi_speed)
        self.mfrc = MFRC522Handler(self.spi)

        gcode = self.printer.lookup_object("gcode")

        # Per-reader debug
        gcode.register_mux_command(
            "AFC_RFID_TAG", "NAME", self.name, self.cmd_AFC_RFID_TAG,
            desc="Read tag on this reader and report UID + spoolman_id if found."
        )

        # Global commands (registered once)
        if not hasattr(self.printer, "_afc_rfid_global_registered"):
            self.printer._afc_rfid_global_registered = True

            # Debug
            gcode.register_command(
                "AFC_RFID_LANES", self.cmd_GLOBAL_AFC_RFID_LANES,
                desc="Show which lanes are mapped to each afc_rfid reader."
            )
            gcode.register_command(
                "AFC_RFID_PENDING", self.cmd_GLOBAL_AFC_RFID_PENDING,
                desc="Show pending RFID spool assignments."
            )

            # Manual one-shot scan+assign
            gcode.register_command(
                "AFC_RFID_SCAN", self.cmd_GLOBAL_AFC_RFID_SCAN,
                desc="Scan RFID for given LANE and immediately assign spool via SET_SPOOL_ID."
            )

            # Two-stage (used by AFC_lane.py prep_callback)
            gcode.register_command(
                "AFC_RFID_SCAN_BEGIN", self.cmd_GLOBAL_AFC_RFID_SCAN_BEGIN,
                desc="Scan RFID for given LANE and store pending result (no assignment yet)."
            )
            gcode.register_command(
                "AFC_RFID_SCAN_COMMIT", self.cmd_GLOBAL_AFC_RFID_SCAN_COMMIT,
                desc="Commit pending RFID scan for LANE using SET_SPOOL_ID, then clear pending."
            )

            # Backward-compatible aliases (in case you already referenced them)
            gcode.register_command(
                "AFC_RFID_BEGIN_LOAD", self.cmd_GLOBAL_AFC_RFID_SCAN_BEGIN,
                desc="(alias) same as AFC_RFID_SCAN_BEGIN"
            )
            gcode.register_command(
                "AFC_RFID_COMMIT_LOAD", self.cmd_GLOBAL_AFC_RFID_SCAN_COMMIT,
                desc="(alias) same as AFC_RFID_SCAN_COMMIT"
            )

    # ----------------- helpers -----------------
    def _normalize_lane(self, lane):
        lane = str(lane).strip().lower()
        if lane.startswith("lane"):
            return lane
        if lane.isdigit():
            return "lane%s" % lane
        return "lane" + lane

    def _all_readers(self):
        readers = []
        pobj = getattr(self.printer, "objects", None)
        if isinstance(pobj, dict):
            for objname, obj in pobj.items():
                if objname.startswith("afc_rfid "):
                    readers.append(obj)

        if not readers:
            # fallback common names
            for n in ("mfrc522_0", "mfrc522_1", "mfrc522_2", "mfrc522_3"):
                try:
                    readers.append(self.printer.lookup_object("afc_rfid %s" % n))
                except Exception:
                    pass
        return readers

    def _find_reader_for_lane(self, lane):
        lane = self._normalize_lane(lane)
        for r in self._all_readers():
            if lane in (getattr(r, "lanes", None) or set()):
                return r, lane
        return None, lane

    def _lane_has_spool_id(self, lane):
        try:
            afc = self.printer.lookup_object("AFC")
            lane_obj = getattr(afc, "lanes", {}).get(lane)
            if lane_obj is None:
                return False
            cur = getattr(lane_obj, "spool_id", None)
            if cur is None:
                return False
            return int(cur) != 0
        except Exception:
            return False

    def _set_lane_spool_id(self, lane, spool_id):
        gcode = self.printer.lookup_object("gcode")
        gcode.run_script_from_command(
            "SET_SPOOL_ID LANE=%s SPOOL_ID=%s" % (lane, int(spool_id))
        )

    def _scan_for_spool(self, tries=10, delay=0.12, max_pages=80):
        last = None
        for _ in range(max(1, int(tries))):
            info = self._read_tag_once(max_pages=max_pages)
            if info:
                last = info
                if info.get("spoolman_id"):
                    return info
            time.sleep(float(delay))
        return last

    # ----------------- NDEF extraction -----------------
    def _extract_ndef_payload(self, raw):
        if not raw:
            return b""
        i = 0
        while i < len(raw):
            t = raw[i]
            if t == 0x00:
                i += 1
                continue
            if t == 0xFE:
                return b""
            if t != 0x03:
                if i + 1 >= len(raw):
                    return b""
                ln = raw[i + 1]
                if ln == 0xFF:
                    if i + 3 >= len(raw):
                        return b""
                    ln2 = (raw[i + 2] << 8) | raw[i + 3]
                    i += 4 + ln2
                else:
                    i += 2 + ln
                continue

            if i + 1 >= len(raw):
                return b""
            ln = raw[i + 1]
            off = i + 2
            if ln == 0xFF:
                if i + 3 >= len(raw):
                    return b""
                ln = (raw[i + 2] << 8) | raw[i + 3]
                off = i + 4
            return raw[off:off + ln]
        return b""

    def _extract_spoolman_id(self, text):
        if not text:
            return None

        # JSON blob
        start = text.find("{")
        end = text.rfind("}")
        if start != -1 and end != -1 and end > start:
            blob = text[start:end + 1]
            try:
                obj = json.loads(blob)
                for k in ("spool_id", "spoolId", "spoolman_id", "spoolmanId", "id"):
                    if k in obj:
                        try:
                            return int(obj[k])
                        except Exception:
                            pass
            except Exception:
                pass

        # key/value
        m = re.search(r"(?:spool_id|spoolId|spoolman_id|spoolmanId)\s*[:=]\s*(\d+)", text)
        if m:
            return int(m.group(1))

        # URL query
        m = re.search(r"[?&](?:spool_id|spoolId|spoolman_id|spoolmanId)=(\d+)", text)
        if m:
            return int(m.group(1))

        if text.strip().isdigit():
            return int(text.strip())

        return None

    # ----------------- tag read -----------------
    def _read_tag_once(self, max_pages=80):
        self.mfrc.antenna_on()
        try:
            uid = self.mfrc.read_uid()
            if not uid:
                return None
            uid_hex = self.mfrc.format_block_data(uid, compact=True)

            raw = self.mfrc.read_ntag_user_bytes_for_uid(uid, max_pages=max_pages) or b""
            payload = self._extract_ndef_payload(raw)
            try:
                text = payload.decode("utf-8", errors="ignore")
            except Exception:
                text = ""
            spoolman_id = self._extract_spoolman_id(text)

            return {
                "uid_hex": uid_hex,
                "spoolman_id": spoolman_id,
                "text": text,
                "raw_len": len(raw),
            }
        finally:
            self.mfrc.antenna_off()

    # ----------------- commands -----------------
    def cmd_AFC_RFID_TAG(self, gcmd):
        tries = int(gcmd.get_int("TRIES", 8))
        delay = float(gcmd.get_float("DELAY", 0.12))
        max_pages = int(gcmd.get_int("MAX_PAGES", 80))

        info = self._scan_for_spool(tries=tries, delay=delay, max_pages=max_pages)
        gcode = self.printer.lookup_object("gcode")
        if not info:
            gcode.respond_info("afc_rfid[%s]: no tag" % self.name)
            return
        txt = (info.get("text", "") or "").replace("\n", " | ")
        if len(txt) > 200:
            txt = txt[:200] + "..."
        gcode.respond_info(
            "afc_rfid[%s]: uid=%s spoolman_id=%s raw_len=%s text=%s" % (
                self.name, info.get("uid_hex", ""), info.get("spoolman_id"),
                info.get("raw_len"), txt
            )
        )

    def cmd_GLOBAL_AFC_RFID_LANES(self, gcmd):
        gcode = self.printer.lookup_object("gcode")
        readers = self._all_readers()
        if not readers:
            gcode.respond_info("AFC_RFID_LANES: no afc_rfid readers found")
            return
        for r in readers:
            gcode.respond_info(
                "afc_rfid[%s]: lanes=%s" % (
                    getattr(r, "name", "?"),
                    sorted(list(getattr(r, "lanes", set()) or set()))
                )
            )

    def cmd_GLOBAL_AFC_RFID_PENDING(self, gcmd):
        gcode = self.printer.lookup_object("gcode")
        anyp = False
        for r in self._all_readers():
            pend = getattr(r, "_pending", {}) or {}
            for lane, p in pend.items():
                anyp = True
                gcode.respond_info(
                    "afc_rfid: pending lane=%s spoolman_id=%s age=%.1fs reader=%s" % (
                        lane,
                        p.get("spoolman_id"),
                        time.time() - float(p.get("ts", 0.0)),
                        getattr(r, "name", "?"),
                    )
                )
        if not anyp:
            gcode.respond_info("afc_rfid: pending (none)")

    def cmd_GLOBAL_AFC_RFID_SCAN(self, gcmd):
        """
        Manual one-shot scan+assign.

        Usage:
          AFC_RFID_SCAN LANE=1
          AFC_RFID_SCAN LANE=lane1

        Options:
          SKIP_IF_SET=0/1 (default 1)
          OVERWRITE=0/1   (default 0) - if 1, forces assignment even if lane already has spool_id
          TRIES=<n>       (default 12)
          DELAY=<sec>     (default 0.12)
          MAX_PAGES=<n>   (default 80)
        """
        lane_in = gcmd.get("LANE", None)
        if not lane_in:
            raise gcmd.error("LANE is required (example: LANE=1 or LANE=lane1)")
        lane = self._normalize_lane(lane_in)

        skip_if_set = bool(gcmd.get_int("SKIP_IF_SET", 1))
        overwrite = bool(gcmd.get_int("OVERWRITE", 0))
        if skip_if_set and not overwrite and self._lane_has_spool_id(lane):
            return

        tries = int(gcmd.get_int("TRIES", 12))
        delay = float(gcmd.get_float("DELAY", 0.12))
        max_pages = int(gcmd.get_int("MAX_PAGES", 80))

        reader, lane = self._find_reader_for_lane(lane)
        if reader is None:
            raise gcmd.error("No afc_rfid reader mapped to %s (check lane: mapping in [afc_rfid ...])" % lane)

        info = reader._scan_for_spool(tries=tries, delay=delay, max_pages=max_pages)
        if not info or not info.get("spoolman_id"):
            return

        self._set_lane_spool_id(lane, int(info["spoolman_id"]))

    def cmd_GLOBAL_AFC_RFID_SCAN_BEGIN(self, gcmd):
        """
        Two-stage scan: begin (store pending).

        Usage:
          AFC_RFID_SCAN_BEGIN LANE=lane1 TIMEOUT=60

        Options:
          SKIP_IF_SET=0/1 (default 1)
          TRIES=<n>       (default 12)
          DELAY=<sec>     (default 0.12)
          MAX_PAGES=<n>   (default 80)
          TIMEOUT=<sec>   (default 60)
        """
        lane_in = gcmd.get("LANE", None)
        if not lane_in:
            raise gcmd.error("LANE is required")
        lane = self._normalize_lane(lane_in)

        skip_if_set = bool(gcmd.get_int("SKIP_IF_SET", 1))
        if skip_if_set and self._lane_has_spool_id(lane):
            return

        tries = int(gcmd.get_int("TRIES", 12))
        delay = float(gcmd.get_float("DELAY", 0.12))
        max_pages = int(gcmd.get_int("MAX_PAGES", 80))
        timeout = float(gcmd.get_float("TIMEOUT", 60.0))

        reader, lane = self._find_reader_for_lane(lane)
        if reader is None:
            return

        info = reader._scan_for_spool(tries=tries, delay=delay, max_pages=max_pages)
        if not info or not info.get("spoolman_id"):
            return

        reader._pending[lane] = {
            "spoolman_id": int(info["spoolman_id"]),
            "uid_hex": info.get("uid_hex", ""),
            "ts": time.time(),
            "timeout": timeout,
        }

    def cmd_GLOBAL_AFC_RFID_SCAN_COMMIT(self, gcmd):
        """
        Two-stage scan: commit (assign pending).

        Usage:
          AFC_RFID_SCAN_COMMIT LANE=lane1

        Options:
          OVERWRITE=0/1 (default 0)
        """
        lane_in = gcmd.get("LANE", None)
        if not lane_in:
            raise gcmd.error("LANE is required")
        lane = self._normalize_lane(lane_in)

        overwrite = bool(gcmd.get_int("OVERWRITE", 0))

        reader, lane = self._find_reader_for_lane(lane)
        if reader is None:
            return

        pend = (reader._pending or {}).get(lane)
        if not pend:
            return

        age = time.time() - float(pend.get("ts", 0.0))
        if age > float(pend.get("timeout", 60.0)):
            reader._pending.pop(lane, None)
            return

        if not overwrite and self._lane_has_spool_id(lane):
            reader._pending.pop(lane, None)
            return

        spool_id = int(pend.get("spoolman_id") or 0)
        if spool_id <= 0:
            reader._pending.pop(lane, None)
            return

        self._set_lane_spool_id(lane, spool_id)
        reader._pending.pop(lane, None)


def load_config_prefix(config):
    return AfcRfid(config)