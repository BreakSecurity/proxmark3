//-----------------------------------------------------------------------------
// Copyright (C) Proxmark3 contributors. See AUTHORS.md for details.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// See LICENSE.txt for the text of the license.
//-----------------------------------------------------------------------------
// readline auto complete utilities
//-----------------------------------------------------------------------------

#ifndef PM3LINE_VOCABULORY_H__
#define PM3LINE_VOCABULORY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct vocabulory_s {
    bool offline;
    const char *name;
} vocabulory_t;

const static vocabulory_t vocabulory[] = {
    { 1, "help" }, 
    { 0, "auto" }, 
    { 1, "clear" }, 
    { 1, "hints" }, 
    { 1, "msleep" }, 
    { 1, "rem" }, 
    { 1, "quit" }, 
    { 1, "exit" }, 
    { 1, "prefs help" }, 
    { 1, "prefs show" }, 
    { 1, "prefs get barmode" }, 
    { 1, "prefs get clientdebug" }, 
    { 1, "prefs get clientdelay" }, 
    { 1, "prefs get color" }, 
    { 1, "prefs get savepaths" }, 
    { 1, "prefs get emoji" }, 
    { 1, "prefs get hints" }, 
    { 1, "prefs get output" }, 
    { 1, "prefs get plotsliders" }, 
    { 1, "prefs set help" }, 
    { 1, "prefs set barmode" }, 
    { 1, "prefs set clientdebug" }, 
    { 1, "prefs set clientdelay" }, 
    { 1, "prefs set color" }, 
    { 1, "prefs set emoji" }, 
    { 1, "prefs set hints" }, 
    { 1, "prefs set savepaths" }, 
    { 1, "prefs set output" }, 
    { 1, "prefs set plotsliders" }, 
    { 1, "analyse help" }, 
    { 1, "analyse lcr" }, 
    { 1, "analyse crc" }, 
    { 1, "analyse chksum" }, 
    { 1, "analyse dates" }, 
    { 1, "analyse lfsr" }, 
    { 1, "analyse a" }, 
    { 1, "analyse nuid" }, 
    { 1, "analyse demodbuff" }, 
    { 1, "analyse freq" }, 
    { 1, "analyse foo" }, 
    { 1, "analyse units" }, 
    { 1, "data help" }, 
    { 1, "data biphaserawdecode" }, 
    { 1, "data detectclock" }, 
    { 1, "data fsktonrz" }, 
    { 1, "data manrawdecode" }, 
    { 1, "data modulation" }, 
    { 1, "data rawdemod" }, 
    { 1, "data askedgedetect" }, 
    { 1, "data autocorr" }, 
    { 1, "data dirthreshold" }, 
    { 1, "data decimate" }, 
    { 1, "data undecimate" }, 
    { 1, "data hide" }, 
    { 1, "data hpf" }, 
    { 1, "data iir" }, 
    { 1, "data grid" }, 
    { 1, "data ltrim" }, 
    { 1, "data mtrim" }, 
    { 1, "data norm" }, 
    { 1, "data plot" }, 
    { 1, "data rtrim" }, 
    { 1, "data setgraphmarkers" }, 
    { 1, "data shiftgraphzero" }, 
    { 1, "data timescale" }, 
    { 1, "data zerocrossings" }, 
    { 1, "data convertbitstream" }, 
    { 1, "data getbitstream" }, 
    { 1, "data asn1" }, 
    { 1, "data bin2hex" }, 
    { 0, "data bitsamples" }, 
    { 1, "data clear" }, 
    { 1, "data diff" }, 
    { 0, "data hexsamples" }, 
    { 1, "data hex2bin" }, 
    { 1, "data load" }, 
    { 1, "data print" }, 
    { 0, "data samples" }, 
    { 1, "data save" }, 
    { 1, "data setdebugmode" }, 
    { 0, "data tune" }, 
    { 1, "emv help" }, 
    { 0, "emv exec" }, 
    { 0, "emv pse" }, 
    { 0, "emv search" }, 
    { 0, "emv select" }, 
    { 0, "emv gpo" }, 
    { 0, "emv readrec" }, 
    { 0, "emv genac" }, 
    { 0, "emv challenge" }, 
    { 0, "emv intauth" }, 
    { 0, "emv scan" }, 
    { 1, "emv test" }, 
    { 1, "emv list" }, 
    { 0, "emv roca" }, 
    { 1, "hf help" }, 
    { 1, "hf list" }, 
    { 0, "hf plot" }, 
    { 0, "hf tune" }, 
    { 1, "hf search" }, 
    { 0, "hf sniff" }, 
    { 1, "hf 14a help" }, 
    { 1, "hf 14a list" }, 
    { 0, "hf 14a antifuzz" }, 
    { 0, "hf 14a config" }, 
    { 0, "hf 14a cuids" }, 
    { 0, "hf 14a info" }, 
    { 0, "hf 14a sim" }, 
    { 0, "hf 14a sniff" }, 
    { 0, "hf 14a raw" }, 
    { 0, "hf 14a reader" }, 
    { 0, "hf 14a apdu" }, 
    { 0, "hf 14a apdufind" }, 
    { 0, "hf 14a chaining" }, 
    { 0, "hf 14a ndefformat" }, 
    { 0, "hf 14a ndefread" }, 
    { 0, "hf 14a ndefwrite" }, 
    { 1, "hf 14b help" }, 
    { 0, "hf 14b apdu" }, 
    { 0, "hf 14b dump" }, 
    { 0, "hf 14b info" }, 
    { 1, "hf 14b list" }, 
    { 0, "hf 14b ndefread" }, 
    { 0, "hf 14b raw" }, 
    { 0, "hf 14b reader" }, 
    { 0, "hf 14b sim" }, 
    { 0, "hf 14b sniff" }, 
    { 0, "hf 14b rdbl" }, 
    { 0, "hf 14b sriwrite" }, 
    { 1, "hf 14b view" }, 
    { 1, "hf 15 help" }, 
    { 1, "hf 15 list" }, 
    { 1, "hf 15 demod" }, 
    { 0, "hf 15 dump" }, 
    { 0, "hf 15 info" }, 
    { 0, "hf 15 sniff" }, 
    { 0, "hf 15 raw" }, 
    { 0, "hf 15 rdbl" }, 
    { 0, "hf 15 rdmulti" }, 
    { 0, "hf 15 reader" }, 
    { 0, "hf 15 restore" }, 
    { 0, "hf 15 samples" }, 
    { 0, "hf 15 eload" }, 
    { 0, "hf 15 esave" }, 
    { 0, "hf 15 eview" }, 
    { 0, "hf 15 sim" }, 
    { 0, "hf 15 slixdisable" }, 
    { 0, "hf 15 wrbl" }, 
    { 0, "hf 15 findafi" }, 
    { 0, "hf 15 writeafi" }, 
    { 0, "hf 15 writedsfid" }, 
    { 0, "hf 15 csetuid" }, 
    { 1, "hf cipurse help" }, 
    { 0, "hf cipurse info" }, 
    { 0, "hf cipurse select" }, 
    { 0, "hf cipurse auth" }, 
    { 0, "hf cipurse read" }, 
    { 0, "hf cipurse write" }, 
    { 0, "hf cipurse aread" }, 
    { 0, "hf cipurse awrite" }, 
    { 0, "hf cipurse formatall" }, 
    { 0, "hf cipurse create" }, 
    { 0, "hf cipurse delete" }, 
    { 0, "hf cipurse updkey" }, 
    { 0, "hf cipurse updakey" }, 
    { 0, "hf cipurse default" }, 
    { 1, "hf cipurse test" }, 
    { 1, "hf epa help" }, 
    { 0, "hf epa cnonces" }, 
    { 0, "hf epa replay" }, 
    { 0, "hf epa sim" }, 
    { 1, "hf emrtd help" }, 
    { 0, "hf emrtd dump" }, 
    { 1, "hf emrtd info" }, 
    { 1, "hf emrtd list" }, 
    { 1, "hf felica help" }, 
    { 1, "hf felica list" }, 
    { 0, "hf felica reader" }, 
    { 0, "hf felica info" }, 
    { 0, "hf felica sniff" }, 
    { 0, "hf felica raw" }, 
    { 0, "hf felica rdbl" }, 
    { 0, "hf felica wrbl" }, 
    { 0, "hf felica rqservice" }, 
    { 0, "hf felica rqresponse" }, 
    { 0, "hf felica scsvcode" }, 
    { 0, "hf felica rqsyscode" }, 
    { 0, "hf felica auth1" }, 
    { 0, "hf felica auth2" }, 
    { 0, "hf felica rqspecver" }, 
    { 0, "hf felica resetmode" }, 
    { 0, "hf felica litesim" }, 
    { 0, "hf felica litedump" }, 
    { 1, "hf fido help" }, 
    { 1, "hf fido list" }, 
    { 0, "hf fido info" }, 
    { 0, "hf fido reg" }, 
    { 0, "hf fido auth" }, 
    { 0, "hf fido make" }, 
    { 0, "hf fido assert" }, 
    { 1, "hf fudan help" }, 
    { 0, "hf fudan reader" }, 
    { 0, "hf fudan dump" }, 
    { 0, "hf fudan rdbl" }, 
    { 1, "hf fudan view" }, 
    { 0, "hf fudan wrbl" }, 
    { 1, "hf gallagher help" }, 
    { 0, "hf gallagher reader" }, 
    { 0, "hf gallagher clone" }, 
    { 0, "hf gallagher delete" }, 
    { 1, "hf gallagher diversifykey" }, 
    { 1, "hf gallagher decode" }, 
    { 1, "hf ksx6924 help" }, 
    { 0, "hf ksx6924 select" }, 
    { 0, "hf ksx6924 info" }, 
    { 0, "hf ksx6924 balance" }, 
    { 0, "hf ksx6924 init" }, 
    { 0, "hf ksx6924 prec" }, 
    { 1, "hf jooki help" }, 
    { 0, "hf jooki clone" }, 
    { 1, "hf jooki decode" }, 
    { 1, "hf jooki encode" }, 
    { 0, "hf jooki sim" }, 
    { 1, "hf iclass help" }, 
    { 0, "hf iclass dump" }, 
    { 1, "hf iclass info" }, 
    { 1, "hf iclass list" }, 
    { 0, "hf iclass rdbl" }, 
    { 0, "hf iclass reader" }, 
    { 0, "hf iclass restore" }, 
    { 0, "hf iclass sniff" }, 
    { 0, "hf iclass wrbl" }, 
    { 0, "hf iclass chk" }, 
    { 1, "hf iclass loclass" }, 
    { 1, "hf iclass lookup" }, 
    { 0, "hf iclass sim" }, 
    { 0, "hf iclass eload" }, 
    { 0, "hf iclass esave" }, 
    { 0, "hf iclass eview" }, 
    { 1, "hf iclass configcard" }, 
    { 1, "hf iclass calcnewkey" }, 
    { 1, "hf iclass encode" }, 
    { 1, "hf iclass encrypt" }, 
    { 1, "hf iclass decrypt" }, 
    { 1, "hf iclass managekeys" }, 
    { 1, "hf iclass permutekey" }, 
    { 1, "hf iclass view" }, 
    { 1, "hf legic help" }, 
    { 0, "hf legic dump" }, 
    { 0, "hf legic info" }, 
    { 1, "hf legic list" }, 
    { 0, "hf legic rdbl" }, 
    { 0, "hf legic reader" }, 
    { 0, "hf legic restore" }, 
    { 0, "hf legic wipe" }, 
    { 0, "hf legic wrbl" }, 
    { 0, "hf legic sim" }, 
    { 0, "hf legic eload" }, 
    { 0, "hf legic esave" }, 
    { 0, "hf legic eview" }, 
    { 1, "hf legic crc" }, 
    { 1, "hf legic view" }, 
    { 1, "hf lto help" }, 
    { 0, "hf lto dump" }, 
    { 0, "hf lto info" }, 
    { 1, "hf lto list" }, 
    { 0, "hf lto rdbl" }, 
    { 0, "hf lto reader" }, 
    { 0, "hf lto restore" }, 
    { 0, "hf lto wrbl" }, 
    { 1, "hf mf help" }, 
    { 1, "hf mf list" }, 
    { 0, "hf mf darkside" }, 
    { 0, "hf mf nested" }, 
    { 1, "hf mf hardnested" }, 
    { 0, "hf mf staticnested" }, 
    { 0, "hf mf autopwn" }, 
    { 0, "hf mf nack" }, 
    { 0, "hf mf chk" }, 
    { 0, "hf mf fchk" }, 
    { 1, "hf mf decrypt" }, 
    { 0, "hf mf supercard" }, 
    { 0, "hf mf auth4" }, 
    { 1, "hf mf acl" }, 
    { 0, "hf mf dump" }, 
    { 1, "hf mf mad" }, 
    { 0, "hf mf personalize" }, 
    { 0, "hf mf rdbl" }, 
    { 0, "hf mf rdsc" }, 
    { 0, "hf mf restore" }, 
    { 0, "hf mf setmod" }, 
    { 1, "hf mf value" }, 
    { 1, "hf mf view" }, 
    { 0, "hf mf wipe" }, 
    { 0, "hf mf wrbl" }, 
    { 0, "hf mf sim" }, 
    { 0, "hf mf ecfill" }, 
    { 0, "hf mf eclr" }, 
    { 0, "hf mf egetblk" }, 
    { 0, "hf mf egetsc" }, 
    { 0, "hf mf ekeyprn" }, 
    { 0, "hf mf eload" }, 
    { 0, "hf mf esave" }, 
    { 0, "hf mf esetblk" }, 
    { 0, "hf mf eview" }, 
    { 0, "hf mf cgetblk" }, 
    { 0, "hf mf cgetsc" }, 
    { 0, "hf mf cload" }, 
    { 0, "hf mf csave" }, 
    { 0, "hf mf csetblk" }, 
    { 0, "hf mf csetuid" }, 
    { 0, "hf mf cview" }, 
    { 0, "hf mf cwipe" }, 
    { 0, "hf mf gen3uid" }, 
    { 0, "hf mf gen3blk" }, 
    { 0, "hf mf gen3freeze" }, 
    { 0, "hf mf ggetblk" }, 
    { 0, "hf mf gload" }, 
    { 0, "hf mf gsetblk" }, 
    { 0, "hf mf gview" }, 
    { 0, "hf mf ndefformat" }, 
    { 0, "hf mf ndefread" }, 
    { 0, "hf mf ndefwrite" }, 
    { 1, "hf mfp help" }, 
    { 0, "hf mfp info" }, 
    { 0, "hf mfp wrp" }, 
    { 0, "hf mfp initp" }, 
    { 0, "hf mfp commitp" }, 
    { 0, "hf mfp auth" }, 
    { 0, "hf mfp rdbl" }, 
    { 0, "hf mfp rdsc" }, 
    { 0, "hf mfp wrbl" }, 
    { 0, "hf mfp chk" }, 
    { 0, "hf mfp mad" }, 
    { 0, "hf mfp ndefread" }, 
    { 1, "hf mfu help" }, 
    { 1, "hf mfu keygen" }, 
    { 1, "hf mfu pwdgen" }, 
    { 0, "hf mfu otptear" }, 
    { 0, "hf mfu cauth" }, 
    { 0, "hf mfu dump" }, 
    { 0, "hf mfu info" }, 
    { 0, "hf mfu ndefread" }, 
    { 0, "hf mfu rdbl" }, 
    { 0, "hf mfu restore" }, 
    { 1, "hf mfu view" }, 
    { 0, "hf mfu wrbl" }, 
    { 0, "hf mfu eload" }, 
    { 0, "hf mfu esave" }, 
    { 0, "hf mfu eview" }, 
    { 0, "hf mfu sim" }, 
    { 0, "hf mfu setpwd" }, 
    { 0, "hf mfu setuid" }, 
    { 1, "hf mfdes help" }, 
    { 0, "hf mfdes info" }, 
    { 0, "hf mfdes getuid" }, 
    { 0, "hf mfdes default" }, 
    { 0, "hf mfdes auth" }, 
    { 0, "hf mfdes chk" }, 
    { 0, "hf mfdes detect" }, 
    { 0, "hf mfdes freemem" }, 
    { 0, "hf mfdes setconfig" }, 
    { 0, "hf mfdes formatpicc" }, 
    { 1, "hf mfdes list" }, 
    { 0, "hf mfdes mad" }, 
    { 0, "hf mfdes lsapp" }, 
    { 0, "hf mfdes getaids" }, 
    { 0, "hf mfdes getappnames" }, 
    { 0, "hf mfdes bruteaid" }, 
    { 0, "hf mfdes createapp" }, 
    { 0, "hf mfdes deleteapp" }, 
    { 0, "hf mfdes selectapp" }, 
    { 0, "hf mfdes changekey" }, 
    { 0, "hf mfdes chkeysettings" }, 
    { 0, "hf mfdes getkeysettings" }, 
    { 0, "hf mfdes getkeyversions" }, 
    { 0, "hf mfdes getfileids" }, 
    { 0, "hf mfdes getfileisoids" }, 
    { 0, "hf mfdes lsfiles" }, 
    { 0, "hf mfdes dump" }, 
    { 0, "hf mfdes createfile" }, 
    { 0, "hf mfdes createvaluefile" }, 
    { 0, "hf mfdes createrecordfile" }, 
    { 0, "hf mfdes createmacfile" }, 
    { 0, "hf mfdes deletefile" }, 
    { 0, "hf mfdes getfilesettings" }, 
    { 0, "hf mfdes chfilesettings" }, 
    { 0, "hf mfdes read" }, 
    { 0, "hf mfdes write" }, 
    { 0, "hf mfdes value" }, 
    { 0, "hf mfdes clearrecfile" }, 
    { 1, "hf mfdes test" }, 
    { 1, "hf ntag424 help" }, 
    { 0, "hf ntag424 info" }, 
    { 0, "hf ntag424 sdm" }, 
    { 1, "hf ntag424 view" }, 
    { 1, "hf seos help" }, 
    { 0, "hf seos info" }, 
    { 1, "hf seos list" }, 
    { 1, "hf st25ta help" }, 
    { 0, "hf st25ta info" }, 
    { 1, "hf st25ta list" }, 
    { 1, "hf st25ta ndefread" }, 
    { 0, "hf st25ta protect" }, 
    { 0, "hf st25ta pwd" }, 
    { 0, "hf st25ta sim" }, 
    { 1, "hf thinfilm help" }, 
    { 0, "hf thinfilm info" }, 
    { 1, "hf thinfilm list" }, 
    { 0, "hf thinfilm sim" }, 
    { 1, "hf topaz help" }, 
    { 0, "hf topaz dump" }, 
    { 1, "hf topaz list" }, 
    { 0, "hf topaz info" }, 
    { 0, "hf topaz reader" }, 
    { 0, "hf topaz sim" }, 
    { 0, "hf topaz sniff" }, 
    { 0, "hf topaz raw" }, 
    { 0, "hf topaz rdbl" }, 
    { 1, "hf topaz view" }, 
    { 0, "hf topaz wrbl" }, 
    { 1, "hf texkom help" }, 
    { 0, "hf texkom reader" }, 
    { 0, "hf texkom sim" }, 
    { 1, "hf xerox help" }, 
    { 0, "hf xerox info" }, 
    { 0, "hf xerox reader" }, 
    { 0, "hf xerox dump" }, 
    { 1, "hf waveshare help" }, 
    { 0, "hf waveshare loadbmp" }, 
    { 1, "hw help" }, 
    { 0, "hw break" }, 
    { 1, "hw connect" }, 
    { 0, "hw dbg" }, 
    { 0, "hw detectreader" }, 
    { 0, "hw fpgaoff" }, 
    { 0, "hw lcd" }, 
    { 0, "hw lcdreset" }, 
    { 0, "hw ping" }, 
    { 0, "hw readmem" }, 
    { 0, "hw reset" }, 
    { 0, "hw setlfdivisor" }, 
    { 0, "hw setmux" }, 
    { 0, "hw standalone" }, 
    { 0, "hw status" }, 
    { 0, "hw tearoff" }, 
    { 0, "hw tia" }, 
    { 0, "hw tune" }, 
    { 1, "hw version" }, 
    { 1, "lf help" }, 
    { 0, "lf config" }, 
    { 0, "lf cmdread" }, 
    { 0, "lf read" }, 
    { 1, "lf search" }, 
    { 0, "lf sim" }, 
    { 0, "lf simask" }, 
    { 0, "lf simfsk" }, 
    { 0, "lf simpsk" }, 
    { 0, "lf simbidir" }, 
    { 0, "lf sniff" }, 
    { 0, "lf tune" }, 
    { 1, "lf awid help" }, 
    { 1, "lf awid demod" }, 
    { 0, "lf awid reader" }, 
    { 0, "lf awid clone" }, 
    { 0, "lf awid sim" }, 
    { 0, "lf awid brute" }, 
    { 0, "lf awid watch" }, 
    { 1, "lf cotag help" }, 
    { 1, "lf cotag demod" }, 
    { 0, "lf cotag reader" }, 
    { 1, "lf destron help" }, 
    { 1, "lf destron demod" }, 
    { 0, "lf destron reader" }, 
    { 0, "lf destron clone" }, 
    { 0, "lf destron sim" }, 
    { 1, "lf em help" }, 
    { 1, "lf em 410x help" }, 
    { 1, "lf em 410x demod" }, 
    { 0, "lf em 410x reader" }, 
    { 0, "lf em 410x sim" }, 
    { 0, "lf em 410x brute" }, 
    { 0, "lf em 410x watch" }, 
    { 0, "lf em 410x spoof" }, 
    { 0, "lf em 410x clone" }, 
    { 1, "lf em 4x05 help" }, 
    { 0, "lf em 4x05 brute" }, 
    { 0, "lf em 4x05 chk" }, 
    { 1, "lf em 4x05 demod" }, 
    { 0, "lf em 4x05 dump" }, 
    { 0, "lf em 4x05 info" }, 
    { 0, "lf em 4x05 read" }, 
    { 1, "lf em 4x05 sniff" }, 
    { 0, "lf em 4x05 unlock" }, 
    { 0, "lf em 4x05 wipe" }, 
    { 0, "lf em 4x05 write" }, 
    { 1, "lf em 4x50 help" }, 
    { 0, "lf em 4x50 brute" }, 
    { 0, "lf em 4x50 chk" }, 
    { 0, "lf em 4x50 dump" }, 
    { 0, "lf em 4x50 info" }, 
    { 0, "lf em 4x50 login" }, 
    { 0, "lf em 4x50 rdbl" }, 
    { 0, "lf em 4x50 reader" }, 
    { 0, "lf em 4x50 restore" }, 
    { 0, "lf em 4x50 wrbl" }, 
    { 0, "lf em 4x50 wrpwd" }, 
    { 0, "lf em 4x50 wipe" }, 
    { 0, "lf em 4x50 eload" }, 
    { 0, "lf em 4x50 esave" }, 
    { 0, "lf em 4x50 eview" }, 
    { 0, "lf em 4x50 sim" }, 
    { 1, "lf em 4x70 help" }, 
    { 0, "lf em 4x70 info" }, 
    { 0, "lf em 4x70 write" }, 
    { 0, "lf em 4x70 unlock" }, 
    { 0, "lf em 4x70 auth" }, 
    { 0, "lf em 4x70 writepin" }, 
    { 0, "lf em 4x70 writekey" }, 
    { 1, "lf fdxb help" }, 
    { 1, "lf fdxb demod" }, 
    { 0, "lf fdxb reader" }, 
    { 0, "lf fdxb clone" }, 
    { 0, "lf fdxb sim" }, 
    { 1, "lf gallagher help" }, 
    { 1, "lf gallagher demod" }, 
    { 0, "lf gallagher reader" }, 
    { 0, "lf gallagher clone" }, 
    { 0, "lf gallagher sim" }, 
    { 1, "lf gproxii help" }, 
    { 1, "lf gproxii demod" }, 
    { 0, "lf gproxii reader" }, 
    { 0, "lf gproxii clone" }, 
    { 0, "lf gproxii sim" }, 
    { 1, "lf hid help" }, 
    { 1, "lf hid demod" }, 
    { 0, "lf hid reader" }, 
    { 0, "lf hid clone" }, 
    { 0, "lf hid sim" }, 
    { 0, "lf hid brute" }, 
    { 0, "lf hid watch" }, 
    { 1, "lf hitag help" }, 
    { 0, "lf hitag eload" }, 
    { 1, "lf hitag list" }, 
    { 0, "lf hitag info" }, 
    { 0, "lf hitag reader" }, 
    { 0, "lf hitag sim" }, 
    { 0, "lf hitag sniff" }, 
    { 0, "lf hitag writer" }, 
    { 0, "lf hitag dump" }, 
    { 0, "lf hitag cc" }, 
    { 1, "lf idteck help" }, 
    { 1, "lf idteck demod" }, 
    { 0, "lf idteck reader" }, 
    { 0, "lf idteck clone" }, 
    { 0, "lf idteck sim" }, 
    { 1, "lf indala help" }, 
    { 0, "lf indala brute" }, 
    { 1, "lf indala demod" }, 
    { 1, "lf indala altdemod" }, 
    { 0, "lf indala reader" }, 
    { 0, "lf indala clone" }, 
    { 0, "lf indala sim" }, 
    { 1, "lf io help" }, 
    { 1, "lf io demod" }, 
    { 0, "lf io reader" }, 
    { 0, "lf io clone" }, 
    { 0, "lf io sim" }, 
    { 0, "lf io watch" }, 
    { 1, "lf jablotron help" }, 
    { 1, "lf jablotron demod" }, 
    { 0, "lf jablotron reader" }, 
    { 0, "lf jablotron clone" }, 
    { 0, "lf jablotron sim" }, 
    { 1, "lf keri help" }, 
    { 1, "lf keri demod" }, 
    { 0, "lf keri reader" }, 
    { 0, "lf keri clone" }, 
    { 0, "lf keri sim" }, 
    { 1, "lf motorola help" }, 
    { 1, "lf motorola demod" }, 
    { 0, "lf motorola reader" }, 
    { 0, "lf motorola clone" }, 
    { 0, "lf motorola sim" }, 
    { 1, "lf nedap help" }, 
    { 1, "lf nedap demod" }, 
    { 0, "lf nedap reader" }, 
    { 0, "lf nedap clone" }, 
    { 0, "lf nedap sim" }, 
    { 1, "lf nexwatch help" }, 
    { 1, "lf nexwatch demod" }, 
    { 0, "lf nexwatch reader" }, 
    { 0, "lf nexwatch clone" }, 
    { 0, "lf nexwatch sim" }, 
    { 1, "lf noralsy help" }, 
    { 1, "lf noralsy demod" }, 
    { 0, "lf noralsy reader" }, 
    { 0, "lf noralsy clone" }, 
    { 0, "lf noralsy sim" }, 
    { 1, "lf pac help" }, 
    { 1, "lf pac demod" }, 
    { 0, "lf pac reader" }, 
    { 0, "lf pac clone" }, 
    { 0, "lf pac sim" }, 
    { 1, "lf paradox help" }, 
    { 1, "lf paradox demod" }, 
    { 0, "lf paradox reader" }, 
    { 0, "lf paradox clone" }, 
    { 0, "lf paradox sim" }, 
    { 1, "lf pcf7931 help" }, 
    { 0, "lf pcf7931 reader" }, 
    { 0, "lf pcf7931 write" }, 
    { 1, "lf pcf7931 config" }, 
    { 1, "lf presco help" }, 
    { 1, "lf presco demod" }, 
    { 0, "lf presco reader" }, 
    { 0, "lf presco clone" }, 
    { 0, "lf presco sim" }, 
    { 1, "lf pyramid help" }, 
    { 1, "lf pyramid demod" }, 
    { 0, "lf pyramid reader" }, 
    { 0, "lf pyramid clone" }, 
    { 0, "lf pyramid sim" }, 
    { 1, "lf securakey help" }, 
    { 1, "lf securakey demod" }, 
    { 0, "lf securakey reader" }, 
    { 0, "lf securakey clone" }, 
    { 0, "lf securakey sim" }, 
    { 1, "lf ti help" }, 
    { 1, "lf ti demod" }, 
    { 0, "lf ti reader" }, 
    { 0, "lf ti write" }, 
    { 1, "lf t55xx help" }, 
    { 0, "lf t55xx clonehelp" }, 
    { 1, "lf t55xx config" }, 
    { 0, "lf t55xx dangerraw" }, 
    { 1, "lf t55xx detect" }, 
    { 0, "lf t55xx deviceconfig" }, 
    { 0, "lf t55xx dump" }, 
    { 1, "lf t55xx info" }, 
    { 0, "lf t55xx p1detect" }, 
    { 0, "lf t55xx read" }, 
    { 0, "lf t55xx resetread" }, 
    { 0, "lf t55xx restore" }, 
    { 1, "lf t55xx trace" }, 
    { 0, "lf t55xx wakeup" }, 
    { 0, "lf t55xx write" }, 
    { 0, "lf t55xx bruteforce" }, 
    { 0, "lf t55xx chk" }, 
    { 0, "lf t55xx protect" }, 
    { 0, "lf t55xx recoverpw" }, 
    { 1, "lf t55xx sniff" }, 
    { 0, "lf t55xx special" }, 
    { 0, "lf t55xx wipe" }, 
    { 1, "lf viking help" }, 
    { 1, "lf viking demod" }, 
    { 0, "lf viking reader" }, 
    { 0, "lf viking clone" }, 
    { 0, "lf viking sim" }, 
    { 1, "lf visa2000 help" }, 
    { 1, "lf visa2000 demod" }, 
    { 0, "lf visa2000 reader" }, 
    { 0, "lf visa2000 clone" }, 
    { 0, "lf visa2000 sim" }, 
    { 1, "mem help" }, 
    { 0, "mem baudrate" }, 
    { 0, "mem dump" }, 
    { 0, "mem info" }, 
    { 0, "mem load" }, 
    { 0, "mem wipe" }, 
    { 1, "mem spiffs help" }, 
    { 0, "mem spiffs copy" }, 
    { 0, "mem spiffs check" }, 
    { 0, "mem spiffs dump" }, 
    { 0, "mem spiffs info" }, 
    { 0, "mem spiffs mount" }, 
    { 0, "mem spiffs remove" }, 
    { 0, "mem spiffs rename" }, 
    { 0, "mem spiffs test" }, 
    { 0, "mem spiffs tree" }, 
    { 0, "mem spiffs unmount" }, 
    { 0, "mem spiffs upload" }, 
    { 0, "mem spiffs view" }, 
    { 0, "mem spiffs wipe" }, 
    { 1, "nfc help" }, 
    { 1, "nfc decode" }, 
    { 0, "nfc type1 read" }, 
    { 1, "nfc type1 help" }, 
    { 0, "nfc type2 read" }, 
    { 1, "nfc type2 help" }, 
    { 0, "nfc type4a format" }, 
    { 0, "nfc type4a read" }, 
    { 0, "nfc type4a write" }, 
    { 0, "nfc type4a st25taread" }, 
    { 1, "nfc type4a help" }, 
    { 0, "nfc type4b read" }, 
    { 1, "nfc type4b help" }, 
    { 0, "nfc mf cformat" }, 
    { 0, "nfc mf cread" }, 
    { 0, "nfc mf cwrite" }, 
    { 0, "nfc mf pread" }, 
    { 1, "nfc mf help" }, 
    { 0, "nfc barcode read" }, 
    { 0, "nfc barcode sim" }, 
    { 1, "nfc barcode help" }, 
    { 1, "smart help" }, 
    { 1, "smart list" }, 
    { 0, "smart info" }, 
    { 0, "smart reader" }, 
    { 0, "smart raw" }, 
    { 1, "smart upgrade" }, 
    { 0, "smart setclock" }, 
    { 0, "smart brute" }, 
    { 1, "script help" }, 
    { 1, "script list" }, 
    { 1, "script run" }, 
    { 1, "trace help" }, 
    { 1, "trace extract" }, 
    { 1, "trace list" }, 
    { 1, "trace load" }, 
    { 1, "trace save" }, 
    { 1, "usart help" }, 
    { 0, "usart btpin" }, 
    { 0, "usart btfactory" }, 
    { 0, "usart tx" }, 
    { 0, "usart rx" }, 
    { 0, "usart txrx" }, 
    { 0, "usart txhex" }, 
    { 0, "usart rxhex" }, 
    { 0, "usart config" }, 
    { 1, "wiegand help" }, 
    { 1, "wiegand list" }, 
    { 1, "wiegand encode" }, 
    { 1, "wiegand decode" }, 
    {0, NULL}
};

#ifdef __cplusplus
}
#endif

#endif