#!/usr/bin/python2
"""
UBX2NMEA

(C) 2010 Timo Juhani Lindfors <timo.lindfors@iki.fi>
(C) 2008 Daniel Willmann <daniel@totalueberwachung.de>
(C) 2008 Openmoko, Inc.
(C) 2018 Petr Holub <hopet@ics.muni.cz>
"""
import struct
import os
import logging
import sys
from datetime import datetime, timedelta
import argparse 
import pprint

import pynmea2

SYNC1=0xb5
SYNC2=0x62

CLASS = {
    "NAV" : 0x01,
    "RXM" : 0x02,
    "INF" : 0x04,
    "ACK" : 0x05,
    "CFG" : 0x06,
    "UPD" : 0x09,
    "MON" : 0x0a,
    "AID" : 0x0b,
    "TIM" : 0x0d,
    "USR" : 0x40
}

CLIDPAIR = {
    "ACK-ACK" : (0x05, 0x01),
    "ACK-NACK" : (0x05, 0x00),
    "AID-ALM" : (0x0b, 0x30),
    "AID-DATA" : (0x0b, 0x10),
    "AID-EPH" : (0x0b, 0x31),
    "AID-HUI" : (0x0b, 0x02),
    "AID-INI" : (0x0b, 0x01),
    "AID-REQ" : (0x0b, 0x00),
    "CFG-ANT" : (0x06, 0x13),
    "CFG-CFG" : (0x06, 0x09),
    "CFG-DAT" : (0x06, 0x06),
    "CFG-EKF" : (0x06, 0x12),
    "CFG-FXN" : (0x06, 0x0e),
    "CFG-INF" : (0x06, 0x02),
    "CFG-LIC" : (0x06, 0x80),
    "CFG-MSG" : (0x06, 0x01),
    "CFG-NAV2" : (0x06, 0x1a),
    "CFG-NMEA" : (0x06, 0x17),
    "CFG-PRT" : (0x06, 0x00),
    "CFG-RATE" : (0x06, 0x08),
    "CFG-RST" : (0x06, 0x04),
    "CFG-RXM" : (0x06, 0x11),
    "CFG-SBAS" : (0x06, 0x16),
    "CFG-TM" : (0x06, 0x10),
    "CFG-TM2" : (0x06, 0x19),
    "CFG-TMODE" : (0x06, 0x1d),
    "CFG-TP" : (0x06, 0x07),
    "CFG-USB" : (0x06, 0x1b),
    "INF-DEBUG" : (0x04, 0x04),
    "INF-ERROR" : (0x04, 0x00),
    "INF-NOTICE" : (0x04, 0x02),
    "INF-TEST" : (0x04, 0x03),
    "INF-USER" : (0x04, 0x07),
    "INF-WARNING" : (0x04, 0x01),
    "MON-EXCEPT" : (0x0a, 0x05),
    "MON-HW" : (0x0a, 0x09),
    "MON-IO" : (0x0a, 0x02),
    "MON-IPC" : (0x0a, 0x03),
    "MON-MSGPP" : (0x0a, 0x06),
    "MON-RXBUF" : (0x0a, 0x07),
    "MON-SCHD" : (0x0a, 0x01),
    "MON-TXBUF" : (0x0a, 0x08),
    "MON-USB" : (0x0a, 0x0a),
    "MON-VER" : (0x0a, 0x04),
    "NAV-CLOCK" : (0x01, 0x22),
    "NAV-DGPS" : (0x01, 0x31),
    "NAV-DOP" : (0x01, 0x04),
    "NAV-EKFSTATUS" : (0x01, 0x40),
    "NAV-POSECEF" : (0x01, 0x01),
    "NAV-POSLLH" : (0x01, 0x02),
    "NAV-POSUTM" : (0x01, 0x08),
    "NAV-SBAS" : (0x01, 0x32),
    "NAV-SOL" : (0x01, 0x06),
    "NAV-STATUS" : (0x01, 0x03),
    "NAV-SVINFO" : (0x01, 0x30),
    "NAV-TIMEGPS" : (0x01, 0x20),
    "NAV-TIMEUTC" : (0x01, 0x21),
    "NAV-VELECEF" : (0x01, 0x11),
    "NAV-VELNED" : (0x01, 0x12),
    "NAV-PVT" : (0x01, 0x07),
    "RXM-ALM" : (0x02, 0x30),
    "RXM-EPH" : (0x02, 0x31),
    "RXM-POSREQ" : (0x02, 0x40),
    "RXM-RAW" : (0x02, 0x10),
    "RXM-SFRB" : (0x02, 0x11),
    "RXM-SVSI" : (0x02, 0x20),
    "TIM-SVIN" : (0x0d, 0x04),
    "TIM-TM" : (0x0d, 0x02),
    "TIM-TM2" : (0x0d, 0x03),
    "TIM-TP" : (0x0d, 0x01),
    "UPD-DOWNL" : (0x09, 0x01),
    "UPD-EXEC" : (0x09, 0x03),
    "UPD-MEMCPY" : (0x09, 0x04),
    "UPD-UPLOAD" : (0x09, 0x02)
}

CLIDPAIR_INV = dict( [ [v,k] for k,v in CLIDPAIR.items() ] )

MSGFMT = {
    ("NAV-POSECEF", 20) :
        ["<IiiiI", ["ITOW", "ECEF_X", "ECEF_Y", "ECEF_Z", "Pacc"]],
    ("NAV-POSLLH", 28) :
        ["<IiiiiII", ["ITOW", "LON", "LAT", "HEIGHT", "HMSL", "Hacc", "Vacc"]],
    ("NAV-POSUTM", 18) :
        ["<Iiiibb", ["ITOW", "EAST", "NORTH", "ALT", "ZONE", "HEM"]],
    ("NAV-DOP", 18) :
        ["<IHHHHHHH", ["ITOW", "GDOP", "PDOP", "TDOP", "VDOP", "HDOP", "NDOP", "EDOP"]],
    ("NAV-STATUS", 16) :
        ["<IBBBxII", ["ITOW", "GPSfix", "Flags", "DiffS", "TTFF", "MSSS"]],
    ("NAV-SOL", 52) :
        ["<IihBBiiiIiiiIHxBxxxx", ["ITOW", "Frac", "week", "GPSFix", "Flags", "ECEF_X", "ECEF_Y", "ECEF_Z", "Pacc",
         "ECEFVX", "ECEFVY", "ECEFVZ", "SAcc", "PDOP", "numSV"]],
    ("NAV-VELECEF", 20) :
        ["<IiiiI", ["ITOW", "ECEFVX", "ECEFVY", "ECEFVZ", "SAcc"]],
    ("NAV-VELNED", 36) :
        ["<IiiiIIiII", ["ITOW", "VEL_N", "VEL_E", "VEL_D", "Speed", "GSpeed", "Heading", "SAcc", "CAcc"]],
    ("NAV-TIMEGPS", 16) :
        ["<IihbBI", ["ITOW", "Frac", "week", "LeapS", "Valid", "TAcc"]],
    ("NAV-PVT", 92) :
        ["<IHBBBBBBLlBBBBllllLLlllllLLHxxxxxxlhH", ["ITOW", "year", "month", "day", "hour", "min", "sec", "valid", "tAcc", "nano", "fixType", "flags", "flags2", "numSV", "lon", "lat", "height", "hMSL", "hAcc", "vAcc", "velN", "velE", "velD", "gSpeed", "headMot", "sAcc", "headAcc", "pDOP", "headVeh", "magDec", "magAcc"]],
    ("NAV-TIMEUTC", 20) :
        ["<IIiHBBBBBB", ["ITOW", "TAcc", "Nano", "Year", "Month", "Day", "Hour", "Min", "Sec", "Valid"]],
    ("NAV-CLOCK",  20) :
        ["<IiiII", ["ITOW", "CLKB", "CLKD", "TAcc", "FAcc"]],
    ("NAV-SVINFO", None) :
        [8, "<IBxxx", ["ITOW", "NCH"], 12, "<BBBbBbhi", ["chn", "SVID", "Flags", "QI", "CNO", "Elev", "Azim", "PRRes"]],
    ("NAV-DGPS", None) :
        [16, "<IihhBBxx", ["ITOW", "AGE", "BASEID", "BASEHLTH", "NCH", "STATUS"], 12, "<BBHff", ["SVID", "Flags", "AGECH", "PRC", "PRRC"]],
    ("NAV-SBAS", None) :
        [12, "<IBBbBBxxx", ["ITOW", "GEO", "MODE", "SYS", "SERVICE", "CNT"], 12, "<BBBBBxhxxh", ["SVID", "FLAGS", "UDRE", "SYSn", "SERVICEn", "PRC", "IC"]],
    ("NAV-EKFSTATUS", 36) : # no response to query
        ["<iiIhbbiiihhhbB", ["pulses", "period", "gyromean", "temp", "dir", "calib", "pulse", "gbias", "gscale", "accps", "accgb", "accgs", "used", "res"]],
    # ('RXM-RAW', [{'Week': 1575, 'ITOW': 475184470, 'NSV': 0}])
    ("RXM-RAW", None) :
        [8, "<ihBx", ["ITOW", "Week", "NSV"], 24, "<ddfBbbB", ["CPMes", "PRMes", "DOMes", "SV", "MesQI", "CNO", "LLI"]],
    ("RXM-SVSI", None) :
        [8, "<ihBB", ["ITOW", "Week", "NumVis", "NumSv"], 6, "<BBhbB", ["SVID", "SVFlag", "Azim", "Elev", "Age"]],
    ("RXM-SFRB", 42) :
        ["<BBiiiiiiiiii", ["CHN", "SVID", "DWRD0", "DWRD1", "DWRD2", "DWRD3", "DWRD4", "DWRD5", "DWRD6", "DWRD7", "DWRD8", "DWRD9"]],
    ("RXM-ALM", 1) :
        ["<B", ["SVID"]],
    ("RXM-ALM", 8)  :
        ["<II", ["SVID", "WEEK"]],
    ("RXM-ALM", 40) :
        ["<" + "I"*10, ["SVID", "WEEK", "DWRD0", "DWRD1", "DWRD2", "DWRD3", "DWRD4", "DWRD5", "DWRD6", "DWRD7"]],
    ("RXM-EPH", 1) :
        ["<B", ["SVID"]],
    ("RXM-EPH", 8) :
        ["<II", ["SVID", "HOW"]],
    ("RXM-EPH", 104) :
        ["<" + "I"*26, ["SVID", "HOW", "SF1D0", "SF1D1", "SF1D2", "SF1D3", "SF1D4",
            "SF1D5", "SF1D6", "SF1D7", "SF2D0", "SF2D1", "SF2D2", "SF2D3", "SF2D4",
            "SF2D5", "SF2D6", "SF2D7", "SF3D0", "SF3D1", "SF3D2", "SF3D3", "SF3D4", "SF3D5", "SF3D6", "SF3D7"]],
    ("INF-ERROR", None) :
        [0, "", [], 1, "c", ["Char"]],
    ("INF-WARNING", None) :
        [0, "", [], 1, "c", ["Char"]],
    ("INF-NOTICE", None) :
        [0, "", [], 1, "c", ["Char"]],
    ("INF-TEST", None) :
        [0, "", [], 1, "c", ["Char"]],
    ("INF-DEBUG", None) :
       [0, "", [], 1, "c", ["Char"]],
    ("INF-USER", None) :
        [0, "", [], 1, "c", ["Char"]],
    ("ACK-ACK", 2) :
        ["<BB", ["ClsID", "MsgID"]],
    ("ACK-NACK", 2) :
        ["<BB", ["ClsID", "MsgID"]],
    ("CFG-PRT", 1) :
        ["<B", ["PortID"]],
    ("CFG-PRT", None) :
        [0, "", [], 20, "<BxxxIIHHHxx", ["PortID", "Mode", "Baudrate", "In_proto_mask", "Out_proto_mask", "Flags"]],
    ("CFG-USB", 108) :
        ["<HHxxHHH32s32s32s", ["VendorID", "ProductID", "reserved2", "PowerConsumption", "Flags", "VendorString", "ProductString", "SerialNumber"]],
    ("CFG-MSG", 2) :
        ["<BB", ["Class", "MsgID"]],
    ("CFG-MSG", 3) :
        ["<BBB", ["Class", "MsgID", "Rate"]],
    ("CFG-NMEA", 4) :
        ["<BBBB", ["Filter", "Version", "NumSV", "Flags"]],
    ("CFG-RATE", 6) :
        ["<HHH", ["Meas", "Nav", "Time"]],
    ("CFG-CFG", 12) :
        ["<III", ["Clear_mask", "Save_mask", "Load_mask"]],
    ("CFG-TP", 20) :
        ["<IIbBxxhhi", ["interval", "length", "status", "time_ref", "antenna_cable_delay", "RF_group_delay", "user_delay"]],
    ("CFG-NAV2", 40) :
        ["<BxxxBBBBiBBBBBBxxHHHHBxxxxxxxxxxx", ["Platform", "MinSVInitial", "MinSVs", "MaxSVs", "FixMode",
         "FixedAltitude", "MinCN0Initial", "MinCN0After", "MinELE", "DGPSTO", "MaxDR", "NAVOPT", "PDOP",
         "TDOP", "PACC", "TACC", "StaticThres"]],
# CFG DAT - Get/Set current Datum
    ("CFG-INF", 1) :
        ["<B", ["ProtocolID"]],
    ("CFG-INF", None) :
        [0, "", [], 8, "<BxxxBBBB", ["ProtocolID", "INFMSG_mask0", "INFMSG_mask1", "INFMSG_mask2", "INFMSG_mask3"]],
    ("CFG-RST", 4) :
        ["<HBx", ["nav_bbr", "Reset"]],
    ("CFG-RXM", 2) :
        ["<BB", ["gps_mode", "lp_mode"]],
    ("CFG-ANT", 4) :
        ["<HH", ["flags", "pins"]],
    ("CFG-FXN", 36) :
        ["<IIIIIIIxxxxI", ["flags", "t_reacq", "t_acq", "t_reacq_off", "t_acq_off", "t_on", "t_off", "base_tow"]],
    ("CFG-SBAS", 8) :
        ["<BBBxI", ["mode", "usage", "maxsbas", "scanmode"]],
    ("CFG-LIC", 12) :
        ["<HHHHHH", ["lic1", "lic2", "lic3", "lic4", "lic5", "lic6"]],
    ("CFG-TM", 12) :
        ["<III", ["INTID", "RATE", "FLAGS"]],
    ("CFG-TM2", 1) :
        ["<B", ["CH"]],
    ("CFG-TM2", 12) :
        ["<BxxxII", ["CH", "RATE", "FLAGS"]],
    ("CFG-TMODE", 28) :
        ["<IiiiIII", ["TimeMode", "FixedPosX", "FixedPosY", "FixedPosZ", "FixedPosVar", "SvinMinDur", "SvinVarLimit"]],
# CFG EKF - Dead Reckoning
# UPD - Lowlevel memory manipulation
    ("UPD-UPLOAD", 12 + 16) :
        ["<III" + "B"*16, ["StartAddr", "DataSize", "Flags", "B0", "B1", "B2", "B3", "B4", "B5", "B6", "B7", "B8", "B9", "B10", "B11", "B12", "B13", "B14", "B15"]],
    ("UPD-UPLOAD", 12 + 1) :
        ["<III" + "B", ["StartAddr", "DataSize", "Flags", "B0"]],
    ("UPD-DOWNL", 8 + 1) :
        ["<II" + "B", ["StartAddr", "Flags", "B0"]],
    ("UPD-DOWNL", 8) :
        ["<II", ["StartAddr", "Flags"]],
    ("MON-SCHD", 24) :
        ["<IIIIHHHBB", ["TSKRUN", "TSKSCHD", "TSKOVRR", "TSKREG", "STACK", "STACKSIZE", "CPUIDLE", "FLYSLY", "PTLSLY"]],
# MON - GPS system statistics
    ("MON-HW", 64 + 8) :
        ["<IIIIHHBBBxI" + ("B" * 32) + "I" + ("x" * 8), ["PinSel", "PinBank", "PinDir", "PinVal", "NoisePerMS", "AGCCnt", "AStatus", "APower", "flags", "useMask", "v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9", "v10", "v11", "v12", "v13", "v14", "v15", "v16", "v17", "v18", "v19", "v20", "v21", "v22", "v23", "v24", "v25", "v26", "v27", "v28", "v29", "v30", "v31", "PinIRQ"]],
    ("MON-VER", 40) :
        ["<30s10s", ["SWVersion", "HWVersion"]],
    ("MON-IPC", 28) :
        ["<I16sII", ["HNDLRINST", "LASTEVENT", "IRQINST", "IRQCALL"]],
    ("MON-EXCEPT", 316) :
        ["<" + ("I" * 79), ["code", "num", "ur0", "ur1", "ur2", "ur3", "ur4", "ur5", "ur6", "ur7", "ur8", "ur9", "ur10", "ur11", "ur12", "usp", "ulr", "fr8", "fr9", "fr10", "fr11", "fr12", "fsp", "flr", "fspsr", "isp", "ilr", "ispsr", "cpsr", "pc", "us0", "us1", "us2", "us3", "us4", "us5", "us6", "us7", "us8", "us9", "us10", "us11", "us12", "us13", "us14", "us15", "res", "is0", "is1", "is2", "is3", "is4", "is5", "is6", "is7", "is8", "is9", "is10", "is11", "is12", "is13", "is14", "is15", "fs0", "fs1", "fs2", "fs3", "fs4", "fs5", "fs6", "fs7", "fs8", "fs9", "fs10", "fs11", "fs12", "fs13", "fs14", "fs15"]],
    ("AID-INI", 48) :
        ["<iiiIHHIiIIiII", ["X", "Y", "Z", "POSACC", "TM_CFG", "WN", "TOW", "TOW_NS", "TACC_MS", "TACC_NS", "CLKD", "CLKDACC", "FLAGS"]],
    ("AID-DATA", 0) :
        ["", []],
    ("AID-HUI", 72) :
        ["<IddiHHHHHHffffffffI", ["HEALTH", "UTC_A1", "UTC_A0", "UTC_TOT", "UTC_WNT",
         "UTC_LS", "UTC_WNF", "UTC_DN", "UTC_LSF", "UTC_SPARE", "KLOB_A0", "KLOB_A1",
         "KLOB_A2", "KLOB_A3", "KLOB_B0", "KLOB_B1", "KLOB_B2", "KLOB_B3", "FLAGS"]],
    ("AID-ALM", 1) :
        ["<B", ["SVID"]],
    ("AID-ALM", 8)  :
        ["<II", ["SVID", "WEEK"]],
    ("AID-ALM", 40) :
        ["<" + "I"*10, ["SVID", "WEEK", "DWRD0", "DWRD1", "DWRD2", "DWRD3", "DWRD4", "DWRD5", "DWRD6", "DWRD7"]],
    ("AID-EPH", 1) :
        ["<B", ["SVID"]],
    ("AID-EPH", 8) :
        ["<II", ["SVID", "HOW"]],
    ("AID-EPH", 104) :
        ["<" + "I"*26, ["SVID", "HOW", "SF1D0", "SF1D1", "SF1D2", "SF1D3", "SF1D4",
            "SF1D5", "SF1D6", "SF1D7", "SF2D0", "SF2D1", "SF2D2", "SF2D3", "SF2D4",
            "SF2D5", "SF2D6", "SF2D7", "SF3D0", "SF3D1", "SF3D2", "SF3D3", "SF3D4", "SF3D5", "SF3D6", "SF3D7"]]
# TIM - Timekeeping
}

ubxToNMEAGGAFix = {
		0 : "0",
		1 : "6",
		2 : "1",
		3 : "1",
		4 : "1", # combined with dead reconing
		}

def ubxToNMEASatNum(version, ubxSVID):
    if (not (version == "2" or version=="3" or version == "4.0" or version == "2ext" or version=="3ext" or version == "4.0ext")):
        raise ValueError("Unsupported NMEA version")

    if (version == "2" or version=="3" or version == "4.0"):
        if (ubxSVID >= 1 and ubxSVID <= 32):
            # normal GPS
            return (None, ubxSVID)
        elif (ubxSVID >= 120 and ubxSVID <= 151):
            # lower SBAS
            return (None, ubxSVID-87)
        elif (ubxSVID >= 65 and ubxSVID <= 96):
            # GLONASS
            return (None, ubxSVID)
        elif (ubxSVID == 255):
            # unknown GLONASS
            return (None, None)
        else:
            return (None, None)
    elif (version == "2ext" or version=="3ext" or version == "4.0ext"):
        if (ubxSVID >= 1 and ubxSVID <= 32):
            # normal GPS
            return (None, ubxSVID)
        elif (ubxSVID >= 120 and ubxSVID <= 151):
            # lower SBAS
            return (None, ubxSVID-87)
        elif (ubxSVID >= 152 and ubxSVID <= 158):
            # upper SBAS
            return (None, ubxSVID)
        elif (ubxSVID >= 65 and ubxSVID <= 96):
            # GLONASS
            return (None, ubxSVID)
        elif (ubxSVID == 255):
            # unknown GLONASS
            return (None, None)
        elif (ubxSVID >= 211 and ubxSVID <= 246):
            # Galileo
            return (None, ubxSVID+90)
        elif (ubxSVID >= 159 and ubxSVID <= 163):
            # upper BeiDou
            return (None, ubxSVID+242)
        elif (ubxSVID >= 33 and ubxSVID <= 64):
            # lower BeiDou
            return (None, ubxSVID+373)
        elif ((ubxSVID >= 173 and ubxSVID <= 182) or (ubxSVID >= 193 and ubxSVID <= 197)):
            # IMES and QZSS
            return (None, ubxSVID)
        else:
            return (None, None)
    elif (version == "4.1"):
        if (ubxSVID >= 1 and ubxSVID <= 32):
            # normal GPS
            return ("GP", ubxSVID)
        elif (ubxSVID >= 120 and ubxSVID <= 151):
            # lower SBAS
            return ("GP", ubxSVID-87)
        elif (ubxSVID >= 65 and ubxSVID <= 96):
            # GLONASS
            return ("GL", ubxSVID)
        elif (ubxSVID == 255):
            # unknown GLONASS
            return ("GL", None)
        elif (ubxSVID >= 211 and ubxSVID <= 246):
            # Galileo
            return ("GA", ubxSVID-210)
        elif (ubxSVID >= 159 and ubxSVID <= 163):
            # upper BeiDou
            return ("GB", ubxSVID-158)
        elif (ubxSVID >= 33 and ubxSVID <= 64):
            # lower BeiDou
            return ("GB", ubxSVID-27)
        else:
            return (None, None)
    elif (version == "4.1ext"):
        if (ubxSVID >= 1 and ubxSVID <= 32):
            # normal GPS
            return ("GP", ubxSVID)
        elif (ubxSVID >= 120 and ubxSVID <= 151):
            # lower SBAS
            return ("GP", ubxSVID-87)
        elif (ubxSVID >= 152 and ubxSVID <= 158):
            # upper SBAS
            return ("GP", ubxSVID)
        elif (ubxSVID >= 65 and ubxSVID <= 96):
            # GLONASS
            return ("GL", ubxSVID)
        elif (ubxSVID == 255):
            # unknown GLONASS
            return ("GL", None)
        elif (ubxSVID >= 211 and ubxSVID <= 246):
            # Galileo
            return ("GA", ubxSVID-210)
        elif (ubxSVID >= 159 and ubxSVID <= 163):
            # upper BeiDou
            return ("GB", ubxSVID-158)
        elif (ubxSVID >= 33 and ubxSVID <= 64):
            # lower BeiDou
            return ("GB", ubxSVID-27)
        elif (ubxSVID >= 173 and ubxSVID <= 182):
            # IMES 
            return (None, ubxSVID)
        elif (ubxSVID >= 193 and ubxSVID <= 197):
            # QZSS
            return ("QZ", ubxSVID)
        else:
            return (None, None)


MSGFMT_INV = dict( [ [(CLIDPAIR[clid], le),v + [clid]] for (clid, le),v in MSGFMT.items() ] )

def decdeg2posdms(dd):
   dd = abs(dd)
   minutes,seconds = divmod(dd*3600,60)
   degrees,minutes = divmod(minutes,60)
   return (degrees,minutes,seconds)

def decdeg2posdm(dd):
   is_positive = dd >= 0
   dd = abs(dd)
   degrees,minutes = divmod(dd,60)
   degrees = degrees if is_positive else -degrees
   return (degrees,minutes)


class Parser():
    def __init__(self, infile, outfile):
        self.fd = open(infile, "rb")
	self.outfd = open(outfile, "w")
        self.buffer = ""
	self.lastsolution = { "ITOW" : None }

    def readAll(self):
        while True:
                data = self.fd.read(4096)
                if not data:
                    break
                #print("read %s" % repr(data))
                self.parse(data)
	self.fd.close()
	self.outfd.close()
        return True

    def parse(self, data):
        self.buffer += data
        buffer_offset = 0
        # Minimum packet length is 8
        while len(self.buffer) >= buffer_offset + 8:
            # Find the beginning of a UBX message
            start = self.buffer.find( chr( SYNC1 ) + chr( SYNC2 ), buffer_offset )

            if buffer_offset == 0 and start != 0:
                #logging.debug( "Discarded data not UBX %s" % repr(self.buffer[:start]) )
                self.buffer = self.buffer[start:]
                continue

            if start == -1 or start + 8 > len(self.buffer):
                return True

            (cl, id, length) = struct.unpack("<BBH", self.buffer[start+2:start+6])
            if len(self.buffer) < start + length + 8:
                buffer_offset = start + 2
                continue

            if self.checksum(self.buffer[start+2:start+length+6]) != struct.unpack("<BB", self.buffer[start+length+6:start+length+8]):
                buffer_offset = start + 2
                continue

            if start != 0:
                logging.warning(" UBX packet ignored %s" % repr(self.buffer[:start]) )
                self.buffer = self.buffer[start:]
                buffer_offset = 0
                continue

            self.decode(cl, id, length, self.buffer[start+6:start+length+6])

            # Discard packet
            self.buffer = self.buffer[start+length+8:]
            buffer_offset = 0

    def checksum( self, msg ):
        ck_a = 0
        ck_b = 0
        for i in msg:
            ck_a = ck_a + ord(i)
            ck_b = ck_b + ck_a
        ck_a = ck_a % 256
        ck_b = ck_b % 256
        return (ck_a, ck_b)

    def decode(self, cl, id, length, payload ):
        data = []
        try:
            format = MSGFMT_INV[((cl, id), length)]
            data.append(dict(zip(format[1], struct.unpack(format[0], payload))))
        except KeyError:
            try:
                # Try if this is one of the variable field messages
                format = MSGFMT_INV[((cl, id), None)]
                fmt_base = format[:3]
                fmt_rep = format[3:]
                # Check if the length matches
                if (length - fmt_base[0])%fmt_rep[0] != 0:
                    logging.error( "Variable length message class 0x%x, id 0x%x \
                        has wrong length %i" % ( cl, id, length ) )
                    return
                data.append(dict(zip(fmt_base[2], struct.unpack(fmt_base[1], payload[:fmt_base[0]]))))
                for i in range(0, (length - fmt_base[0])/fmt_rep[0]):
                    offset = fmt_base[0] + fmt_rep[0] * i
                    data.append(dict(zip(fmt_rep[2], struct.unpack(fmt_rep[1], payload[offset:offset+fmt_rep[0]]))))

            except KeyError:
                logging.info( "Unknown message class 0x%x, id 0x%x, length %i" % ( cl, id, length ) )
                return

        logging.debug( "Got UBX packet of type %s: %s" % (format[-1] , data ) )
        if (not "ITOW" in data[0]):
            return
        if (self.lastsolution["ITOW"] == None):
            self.lastsolution["ITOW"] = data[0]["ITOW"]
        if (self.lastsolution["ITOW"] != data[0]["ITOW"]):
            # dump output for current ITOW here
            if ("NAV-PVT" in self.lastsolution):
                currentTime = datetime.strptime("%04d-%02d-%02d %02d:%02d:%02d" % (self.lastsolution["NAV-PVT"]["year"], self.lastsolution["NAV-PVT"]["month"], self.lastsolution["NAV-PVT"]["day"], self.lastsolution["NAV-PVT"]["hour"], self.lastsolution["NAV-PVT"]["min"], self.lastsolution["NAV-PVT"]["sec"]), "%Y-%m-%d %H:%M:%S")
                currentTime += timedelta(microseconds = self.lastsolution["NAV-PVT"]["nano"] / 1000)
                lon = 1.0 * self.lastsolution["NAV-PVT"]["lon"] / 10**7
                lat = 1.0 * self.lastsolution["NAV-PVT"]["lat"] / 10**7
                pp.pprint(self.lastsolution)
                if ("NAV-DOP" in self.lastsolution):
                    hDOP = 1.0*self.lastsolution["NAV-DOP"]["hDOP"]
                    logging.debug("Using proper hDOP " + str(hDOP))
                else:
                    hDOP = 1.0*self.lastsolution["NAV-PVT"]["pDOP"]
                    logging.debug("Using fake pDOP " + str(hDOP))
                hDOP /= 100
                if (self.lastsolution["NAV-PVT"]["fixType"] != "" and self.lastsolution["NAV-PVT"]["fixType"] in ubxToNMEAGGAFix):
                    self.outfd.write(str(pynmea2.GGA('GN', 'GGA', (currentTime.strftime("%H%M%S") + ".%02d" % (currentTime.microsecond/10000), "%02d%02.5f" % (divmod(abs(lat)*60, 60.0)), ('S','N')[lat>=0.0], "%03d%02.5f" % (divmod(abs(lon)*60, 60.0)), ('W','E')[lon>=0.0], ubxToNMEAGGAFix[self.lastsolution["NAV-PVT"]["fixType"]], "%02d" % self.lastsolution["NAV-PVT"]["numSV"], "%01.1f" % (hDOP), "%.1f" % (1.0*self.lastsolution["NAV-PVT"]["hMSL"]/1000), 'M', "%.1f" % (1.0*self.lastsolution["NAV-PVT"]["height"]/1000), 'M', '', '0000'))) + "\n")
                    self.outfd.write(str(pynmea2.GGA('GN', 'ZDA', (currentTime.strftime("%H%M%S") + ".%02d" % (currentTime.microsecond/10000), "%02d" % self.lastsolution["NAV-PVT"]["day"], "%02d" % self.lastsolution["NAV-PVT"]["month"], "%04d" % self.lastsolution["NAV-PVT"]["year"],"",""))) + "\n")
                    self.outfd.write(str(pynmea2.GGA('GN', 'RMC', (currentTime.strftime("%H%M%S") + ".%02d" % (currentTime.microsecond/10000), ('A', 'V')[ubxToNMEAGGAFix[self.lastsolution["NAV-PVT"]["fixType"]] == "0"], "%02d%02.5f" % (divmod(abs(lat)*60, 60.0)), ('S','N')[lat>=0.0], "%03d%02.5f" % (divmod(abs(lon)*60, 60.0)), ('W','E')[lon>=0.0], "%03.1f" % (1.0 * self.lastsolution["NAV-PVT"]["gSpeed"] / 514.444), "%03.1f" % (1.0 * self.lastsolution["NAV-PVT"]["headMot"]/10**5), "%02d%02d%s" % (self.lastsolution["NAV-PVT"]["day"], self.lastsolution["NAV-PVT"]["month"], str(self.lastsolution["NAV-PVT"]["year"])[-2:]), "%03.1f" % abs(self.lastsolution["NAV-PVT"]["magDec"]), ('E', 'W')[self.lastsolution["NAV-PVT"]["magDec"]>=0.0] ))) + "\n")
            if ("NAV-SVINFO" in self.lastsolution):
                reportedSVsbyConstellation = {}
                # this is preparation for NMEA 4.x reporting
                for i in xrange(1, len(self.lastsolution["NAV-SVINFO"])):
                    (talkerID, satID) = ubxToNMEASatNum("2ext", self.lastsolution["NAV-SVINFO"][i]["SVID"])
                    if (talkerID == None): talkerID = "None"
                    pp.pprint(self.lastsolution["NAV-SVINFO"][i])
                    pp.pprint((talkerID, satID))
                    if (not talkerID in reportedSVsbyConstellation): reportedSVsbyConstellation[talkerID] = {}
                    if (satID != None): reportedSVsbyConstellation[talkerID][satID] = i
                for con in sorted(reportedSVsbyConstellation):
                    satsToReport = len(reportedSVsbyConstellation[con])
                    NMEAcon = (con, '')[con == "None"]
                    for i in sorted(reportedSVsbyConstellation[con]):
                        j = reportedSVsbyConstellation[con][i]
                        if (self.lastsolution["NAV-SVINFO"][j]["QI"] < 4): continue
                        self.outfd.write(str(pynmea2.GSV(NMEAcon, 'GSV', ("1", "1", "%d" % self.lastsolution["NAV-PVT"]["numSV"], "%02d" % i, "%-02d" % self.lastsolution["NAV-SVINFO"][j]["Elev"], "%03d" % self.lastsolution["NAV-SVINFO"][j]["Azim"], "%02d" % self.lastsolution["NAV-SVINFO"][j]["CNO"]))) + "\n")
            # end of dump
            self.lastsolution = {}
            self.lastsolution["ITOW"] = data[0]["ITOW"]
        if (format[-1] == "NAV-SVINFO" or format[-1] == "RXM-SVSI"):
            self.lastsolution[format[-1]] = data
            #pass
        else:
            self.lastsolution[format[-1]] = data[0]


# main code

pp = pprint.PrettyPrinter(indent=4)

navmsgList = ['NAV-PVT']

parser = argparse.ArgumentParser()
parser.add_argument('-v', '--verbose', dest='verbose', action='store_true', help='verbose information on progress')
parser.add_argument('-d', '--debug', dest='debug', action='store_true', help='debug information on progress')
parser.add_argument('-G', '--output-GPX', dest='outputGPX', action='store_true', help='output GPX, not yet implemented (default: disabled)')
parser.add_argument('-N', '--navigation-message', dest='navmsg', nargs=1, choices=navmsgList, help='navigation message to use') 
parser.add_argument('infile', help="Input file name", type=str)
parser.add_argument('outfile', help="Output file name", type=str)
args = parser.parse_args()


if args.debug:
    logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.DEBUG)
elif args.verbose:
    logging.basicConfig(format="%(levelname)s: %(message)s", level=logging.INFO)
else:
    logging.basicConfig(format="%(levelname)s: %(message)s") 

ubxParser = Parser(args.infile, args.outfile)
ubxParser.readAll()
