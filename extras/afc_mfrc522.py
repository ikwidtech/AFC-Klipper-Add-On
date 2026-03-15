import time
from contextlib import contextmanager


class MFRC522Handler:
    # Registers
    CommandReg = 0x01
    ComIEnReg = 0x02
    DivIEnReg = 0x03
    ComIrqReg = 0x04
    DivIrqReg = 0x05
    ErrorReg = 0x06
    Status1Reg = 0x07
    Status2Reg = 0x08
    FIFODataReg = 0x09
    FIFOLevelReg = 0x0A
    WaterLevelReg = 0x0B
    ControlReg = 0x0C
    BitFramingReg = 0x0D
    CollReg = 0x0E
    ModeReg = 0x11
    TxModeReg = 0x12
    RxModeReg = 0x13
    TxControlReg = 0x14
    TxASKReg = 0x15
    RFCfgReg = 0x26
    CRCResultRegH = 0x21
    CRCResultRegL = 0x22
    TModeReg = 0x2A
    TPrescalerReg = 0x2B
    TReloadRegH = 0x2C
    TReloadRegL = 0x2D
    VersionReg = 0x37

    # Commands
    PCD_IDLE = 0x00
    PCD_MEM = 0x01
    PCD_CALCCRC = 0x03
    PCD_TRANSCEIVE = 0x0C
    PCD_SOFTRESET = 0x0F

    # PICC commands
    PICC_REQA = 0x26
    PICC_WUPA = 0x52
    PICC_READ = 0x30  # Type2 / Ultralight / NTAG

    MI_OK = 0
    MI_NOTAGERR = 1
    MI_ERR = 2

    VALID_VERSION_VALUES = (0x88, 0x90, 0x91, 0x92, 0xA2, 0xB2)

    def __init__(self, spi):
        self.spi = spi
        self.timeout_cmd_exec = 0.06
        self._initialized = False

    # ---------- SPI ----------
    def _reg_addr_wr(self, reg):
        return ((reg << 1) & 0x7E)

    def _reg_addr_rd(self, reg):
        return (((reg << 1) & 0x7E) | 0x80)

    def _write_reg_raw(self, reg, val):
        self.spi.spi_send([self._reg_addr_wr(reg), val & 0xFF])

    def _read_reg_raw(self, reg):
        ret = self.spi.spi_transfer([self._reg_addr_rd(reg), 0x00])
        if isinstance(ret, dict) and "response" in ret:
            data = bytearray(ret["response"])
        else:
            data = bytearray(ret)
        if len(data) < 2:
            return 0xFF
        return data[1] & 0xFF

    def _set_bitmask(self, reg, mask):
        self._write_reg_raw(reg, self._read_reg_raw(reg) | (mask & 0xFF))

    def _clear_bitmask(self, reg, mask):
        self._write_reg_raw(reg, self._read_reg_raw(reg) & (~mask & 0xFF))

    # ---------- init ----------
    def _ensure_init(self):
        if self._initialized:
            return
        self.reset()
        self.init()
        self._initialized = True

    def reset(self):
        self._write_reg_raw(self.CommandReg, self.PCD_SOFTRESET)
        time.sleep(0.05)

    def init(self):
        # Stable defaults used by common MFRC522 libraries
        self._write_reg_raw(self.TModeReg, 0x8D)
        self._write_reg_raw(self.TPrescalerReg, 0x3E)
        self._write_reg_raw(self.TReloadRegH, 0x00)
        self._write_reg_raw(self.TReloadRegL, 0x1E)
        self._write_reg_raw(self.TxASKReg, 0x40)
        self._write_reg_raw(self.ModeReg, 0x3D)
        # max Rx gain
        self._write_reg_raw(self.RFCfgReg, 0x70)
        self.antenna_on()

    def read_version_raw(self):
        return self._read_reg_raw(self.VersionReg) & 0xFF

    def read_version(self):
        self._ensure_init()
        return self._read_reg_raw(self.VersionReg) & 0xFF

    def antenna_on(self):
        self._set_bitmask(self.TxControlReg, 0x03)

    def antenna_off(self):
        self._clear_bitmask(self.TxControlReg, 0x03)

    @contextmanager
    def antenna_manager(self):
        self._ensure_init()
        self.antenna_on()
        try:
            yield
        finally:
            pass

    # ---------- CRC ----------
    def _crc_a(self, data):
        self._ensure_init()
        self._write_reg_raw(self.CommandReg, self.PCD_IDLE)
        self._write_reg_raw(self.DivIrqReg, 0x04)
        self._write_reg_raw(self.FIFOLevelReg, 0x80)
        for b in data:
            self._write_reg_raw(self.FIFODataReg, b)
        self._write_reg_raw(self.CommandReg, self.PCD_CALCCRC)
        start = time.time()
        while True:
            n = self._read_reg_raw(self.DivIrqReg)
            if n & 0x04:
                break
            if time.time() - start > self.timeout_cmd_exec:
                return [0x00, 0x00]
        return [
            self._read_reg_raw(self.CRCResultRegL) & 0xFF,
            self._read_reg_raw(self.CRCResultRegH) & 0xFF,
        ]

    # ---------- transceive ----------
    def _clear_irqs(self):
        self._write_reg_raw(self.ComIrqReg, 0x7F)
        self._write_reg_raw(self.DivIrqReg, 0x7F)

    def to_card(self, command, send_data, tx_last_bits=0):
        self._ensure_init()
        irq_en = 0x00
        wait_irq = 0x00
        if command == self.PCD_TRANSCEIVE:
            irq_en = 0x77
            wait_irq = 0x30

        self._write_reg_raw(self.ComIEnReg, (irq_en | 0x80) & 0xFF)
        self._clear_irqs()
        self._write_reg_raw(self.FIFOLevelReg, 0x80)
        self._write_reg_raw(self.CommandReg, self.PCD_IDLE)

        for b in send_data:
            self._write_reg_raw(self.FIFODataReg, b)

        self._write_reg_raw(self.BitFramingReg, tx_last_bits & 0x07)
        self._write_reg_raw(self.CommandReg, command)
        if command == self.PCD_TRANSCEIVE:
            self._set_bitmask(self.BitFramingReg, 0x80)

        start = time.time()
        while True:
            n = self._read_reg_raw(self.ComIrqReg)
            if n & wait_irq:
                break
            if n & 0x01:
                break
            if time.time() - start > self.timeout_cmd_exec:
                break

        self._clear_bitmask(self.BitFramingReg, 0x80)

        err = self._read_reg_raw(self.ErrorReg)
        if err & 0x13:
            return (self.MI_ERR, [], 0)

        status = self.MI_OK
        if n & 0x01:
            status = self.MI_NOTAGERR

        back_data = []
        back_bits = 0
        if command == self.PCD_TRANSCEIVE:
            fifo_level = self._read_reg_raw(self.FIFOLevelReg)
            last_bits = self._read_reg_raw(self.ControlReg) & 0x07
            if last_bits:
                back_bits = (fifo_level - 1) * 8 + last_bits
            else:
                back_bits = fifo_level * 8
            if fifo_level == 0:
                fifo_level = 1
            if fifo_level > 64:
                fifo_level = 64
            for _ in range(fifo_level):
                back_data.append(self._read_reg_raw(self.FIFODataReg))

        return (status, back_data, back_bits)

    # ---------- ISO14443A ----------
    def request(self, req_mode=PICC_REQA):
        self._ensure_init()
        self._write_reg_raw(self.BitFramingReg, 0x07)
        status, back_data, back_bits = self.to_card(self.PCD_TRANSCEIVE, [req_mode], tx_last_bits=7)
        if status != self.MI_OK or back_bits != 0x10:
            return (self.MI_ERR, None)
        return (self.MI_OK, back_data)

    def _anticoll_level(self, sel):
        self._write_reg_raw(self.BitFramingReg, 0x00)
        self._write_reg_raw(self.CollReg, 0x80)
        status, back_data, _ = self.to_card(self.PCD_TRANSCEIVE, [sel, 0x20])
        if status != self.MI_OK or len(back_data) < 5:
            return (self.MI_ERR, None)
        back_data = back_data[:5]
        chk = 0
        for b in back_data[:4]:
            chk ^= b
        if chk != back_data[4]:
            return (self.MI_ERR, None)
        return (self.MI_OK, back_data)

    def _select_level(self, sel, uid_cln):
        buf = [sel, 0x70] + uid_cln[:5]
        buf += self._crc_a(buf)
        status, back_data, back_bits = self.to_card(self.PCD_TRANSCEIVE, buf)
        if status != self.MI_OK or back_bits != 0x18 or len(back_data) < 1:
            return (self.MI_ERR, None)
        return (self.MI_OK, back_data[0] & 0xFF)  # SAK

    def read_uid(self):
        self._ensure_init()
        uid = []
        with self.antenna_manager():
            st, _ = self.request(self.PICC_REQA)
            if st != self.MI_OK:
                return None
            for sel in (0x93, 0x95, 0x97):
                st, cln = self._anticoll_level(sel)
                if st != self.MI_OK:
                    return None
                st, sak = self._select_level(sel, cln)
                if st != self.MI_OK:
                    return None
                if cln[0] == 0x88:
                    uid.extend(cln[1:4])
                else:
                    uid.extend(cln[0:4])
                if not (sak & 0x04):
                    return uid
        return uid or None

    # ---------- NTAG / Type 2 ----------
    def _tag_read_4pages(self, start_page):
        cmd = [self.PICC_READ, start_page & 0xFF]
        cmd += self._crc_a(cmd)
        status, back_data, back_bits = self.to_card(self.PCD_TRANSCEIVE, cmd)
        if status != self.MI_OK or back_bits < 16 * 8 or len(back_data) < 16:
            return None
        return back_data[:16]

    def read_ntag_user_bytes(self, max_pages=40):
        uid = self.read_uid()
        if not uid:
            return None
        return self.read_ntag_user_bytes_for_uid(uid, max_pages=max_pages)

    def read_ntag_user_bytes_for_uid(self, uid, max_pages=40):
        # NOTE: MFRC522 Type2 READ does not require authentication for NTAG21x.
        # User memory starts at page 4.
        if not uid:
            return None
        data = []
        for page in range(4, 4 + max_pages, 4):
            block = self._tag_read_4pages(page)
            if not block:
                break
            data.extend(block)
        return bytes(data)

    # ---------- NDEF parsing helpers ----------
    def _parse_ndef(self, raw):
        if not raw:
            return None
        i = 0
        while i < len(raw):
            t = raw[i]
            if t == 0x00:
                i += 1
                continue
            if t == 0xFE:
                break
            if t != 0x03:
                if i + 1 >= len(raw):
                    break
                ln = raw[i + 1]
                if ln == 0xFF and i + 3 < len(raw):
                    ln = (raw[i + 2] << 8) | raw[i + 3]
                    i += 4 + ln
                else:
                    i += 2 + ln
                continue
            if i + 1 >= len(raw):
                return None
            ln = raw[i + 1]
            off = i + 2
            if ln == 0xFF:
                if i + 3 >= len(raw):
                    return None
                ln = (raw[i + 2] << 8) | raw[i + 3]
                off = i + 4
            return raw[off:off + ln]
        return None

    def parse_ndef_records(self, raw):
        payload = self._parse_ndef(raw)
        if not payload:
            return []
        recs = []
        i = 0
        while i < len(payload):
            hdr = payload[i]
            i += 1
            if i >= len(payload):
                break
            sr = bool(hdr & 0x10)
            il = bool(hdr & 0x08)
            tnf = hdr & 0x07
            type_len = payload[i]
            i += 1
            if sr:
                if i >= len(payload):
                    break
                pl_len = payload[i]
                i += 1
            else:
                if i + 3 >= len(payload):
                    break
                pl_len = (payload[i] << 24) | (payload[i + 1] << 16) | (payload[i + 2] << 8) | payload[i + 3]
                i += 4
            id_len = 0
            if il:
                if i >= len(payload):
                    break
                id_len = payload[i]
                i += 1
            rtype = bytes(payload[i:i + type_len])
            i += type_len
            rid = bytes(payload[i:i + id_len])
            i += id_len
            rpayload = bytes(payload[i:i + pl_len])
            i += pl_len
            recs.append({"tnf": tnf, "type": rtype, "id": rid, "payload": rpayload})
            if hdr & 0x40:
                break
        return recs

    def extract_tag_text(self, raw):
        recs = self.parse_ndef_records(raw)
        texts = []
        for rec in recs:
            rtype = rec["type"]
            payload = rec["payload"]
            if rec["tnf"] == 0x01 and rtype == b"T" and payload:
                status = payload[0]
                lang_len = status & 0x3F
                txt = payload[1 + lang_len:]
                try:
                    texts.append(txt.decode("utf-8", errors="ignore"))
                except Exception:
                    pass
            elif rec["tnf"] == 0x01 and rtype == b"U" and payload:
                prefixes = ["", "http://www.", "https://www.", "http://", "https://"]
                prefix = prefixes[payload[0]] if payload[0] < len(prefixes) else ""
                try:
                    texts.append(prefix + payload[1:].decode("utf-8", errors="ignore"))
                except Exception:
                    pass
            else:
                try:
                    texts.append(payload.decode("utf-8", errors="ignore"))
                except Exception:
                    pass
        if not texts and raw:
            try:
                texts.append(raw.decode("utf-8", errors="ignore"))
            except Exception:
                pass
        return "\n".join([t for t in texts if t])

    def extract_spoolman_id(self, raw):
        import json
        import re
        text = self.extract_tag_text(raw)
        if not text:
            return None, text

        candidates = []
        start = text.find("{")
        while start != -1:
            end = text.find("}", start)
            if end == -1:
                break
            candidates.append(text[start:end + 1])
            start = text.find("{", start + 1)
        for c in candidates:
            try:
                obj = json.loads(c)
                for key in ("spool_id", "spoolman_id", "spoolId", "id"):
                    if key in obj:
                        try:
                            return int(obj[key]), text
                        except Exception:
                            pass
            except Exception:
                pass

        m = re.search(r'(?:spool_id|spoolman_id|spoolId)\s*[=:]\s*(\d+)', text)
        if m:
            return int(m.group(1)), text
        m = re.search(r'[?&](?:spool_id|spoolman_id|spoolId)=(\d+)', text)
        if m:
            return int(m.group(1)), text

        stripped = text.strip()
        if stripped.isdigit():
            return int(stripped), text

        return None, text

    # ---------- helpers ----------
    def read_tag_info(self, max_pages=40):
        uid = self.read_uid()
        if not uid:
            return None
        raw = self.read_ntag_user_bytes_for_uid(uid, max_pages=max_pages)
        spool_id, tag_text = self.extract_spoolman_id(raw or b"")
        return {
            "uid": uid,
            "uid_hex": self.format_block_data(uid, compact=True),
            "raw_len": len(raw) if raw else 0,
            "spoolman_id": spool_id,
            "tag_text": tag_text,
        }

    def format_block_data(self, uid, compact=False):
        if uid is None:
            return ""
        if compact:
            return "".join(["%02X" % (b & 0xFF) for b in uid])
        return " ".join(["%02X" % (b & 0xFF) for b in uid])