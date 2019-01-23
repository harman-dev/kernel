/*COPYRIGHT**
    Copyright (C) 2005-2015 Intel Corporation.  All Rights Reserved.

    This file is part of SEP Development Kit

    SEP Development Kit is free software; you can redistribute it
    and/or modify it under the terms of the GNU General Public License
    version 2 as published by the Free Software Foundation.

    SEP Development Kit is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SEP Development Kit; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

    As a special exception, you may use this file as part of a free software
    library without restriction.  Specifically, if other files instantiate
    templates or use macros or inline functions from this file, or you compile
    this file and link it with other files to produce an executable, this
    file does not by itself cause the resulting executable to be covered by
    the GNU General Public License.  This exception does not however
    invalidate any other reasons why the executable file might be covered by
    the GNU General Public License.
**COPYRIGHT*/

#ifndef _KNL_CHA_H_INC_
#define _KNL_CHA_H_INC_


#define KNL_CHA0_BOX_CTL_MSR                     0xE00
#define KNL_CHA1_BOX_CTL_MSR                     0xE0C
#define KNL_CHA2_BOX_CTL_MSR                     0xE18
#define KNL_CHA3_BOX_CTL_MSR                     0xE24
#define KNL_CHA4_BOX_CTL_MSR                     0xE30
#define KNL_CHA5_BOX_CTL_MSR                     0xE3C
#define KNL_CHA6_BOX_CTL_MSR                     0xE48
#define KNL_CHA7_BOX_CTL_MSR                     0xE54
#define KNL_CHA8_BOX_CTL_MSR                     0xE60
#define KNL_CHA9_BOX_CTL_MSR                     0xE6C
#define KNL_CHA10_BOX_CTL_MSR                    0xE78
#define KNL_CHA11_BOX_CTL_MSR                    0xE84
#define KNL_CHA12_BOX_CTL_MSR                    0xE90
#define KNL_CHA13_BOX_CTL_MSR                    0xE9C
#define KNL_CHA14_BOX_CTL_MSR                    0xEA8
#define KNL_CHA15_BOX_CTL_MSR                    0xEB4
#define KNL_CHA16_BOX_CTL_MSR                    0xEC0
#define KNL_CHA17_BOX_CTL_MSR                    0xECC
#define KNL_CHA18_BOX_CTL_MSR                    0xED8
#define KNL_CHA19_BOX_CTL_MSR                    0xEE4
#define KNL_CHA20_BOX_CTL_MSR                    0xEF0
#define KNL_CHA21_BOX_CTL_MSR                    0xEFC
#define KNL_CHA22_BOX_CTL_MSR                    0xF08
#define KNL_CHA23_BOX_CTL_MSR                    0xF14
#define KNL_CHA24_BOX_CTL_MSR                    0xF20
#define KNL_CHA25_BOX_CTL_MSR                    0xF2C
#define KNL_CHA26_BOX_CTL_MSR                    0xF38
#define KNL_CHA27_BOX_CTL_MSR                    0xF44
#define KNL_CHA28_BOX_CTL_MSR                    0xE50
#define KNL_CHA29_BOX_CTL_MSR                    0xF5C
#define KNL_CHA30_BOX_CTL_MSR                    0xF68
#define KNL_CHA31_BOX_CTL_MSR                    0xF74
#define KNL_CHA32_BOX_CTL_MSR                    0xF80
#define KNL_CHA33_BOX_CTL_MSR                    0xF8C
#define KNL_CHA34_BOX_CTL_MSR                    0xF98
#define KNL_CHA35_BOX_CTL_MSR                    0xFA4
#define KNL_CHA36_BOX_CTL_MSR                    0xFB0
#define KNL_CHA37_BOX_CTL_MSR                    0xFBC

#define KNL_CHA37_PMON_CTL0_MSR                  0xFBD
#define KNL_CHA37_PMON_CTL3_MSR                  0xFC0
#define KNL_CHA36_PMON_CTL0_MSR                  0xFB1
#define KNL_CHA36_PMON_CTL3_MSR                  0xFB4
#define KNL_CHA35_PMON_CTL0_MSR                  0xFA5
#define KNL_CHA35_PMON_CTL3_MSR                  0xFA8
#define KNL_CHA34_PMON_CTL0_MSR                  0xF99
#define KNL_CHA34_PMON_CTL3_MSR                  0xF9C
#define KNL_CHA33_PMON_CTL0_MSR                  0xF8D
#define KNL_CHA33_PMON_CTL3_MSR                  0xF90
#define KNL_CHA32_PMON_CTL0_MSR                  0xF81
#define KNL_CHA32_PMON_CTL3_MSR                  0xF84
#define KNL_CHA31_PMON_CTL0_MSR                  0xF75
#define KNL_CHA31_PMON_CTL3_MSR                  0xF78
#define KNL_CHA30_PMON_CTL0_MSR                  0xF69
#define KNL_CHA30_PMON_CTL3_MSR                  0xF6C
#define KNL_CHA29_PMON_CTL0_MSR                  0xF5D
#define KNL_CHA29_PMON_CTL3_MSR                  0xF60
#define KNL_CHA28_PMON_CTL0_MSR                  0xF51
#define KNL_CHA28_PMON_CTL3_MSR                  0xF54
#define KNL_CHA27_PMON_CTL0_MSR                  0xF45
#define KNL_CHA27_PMON_CTL3_MSR                  0xF48
#define KNL_CHA26_PMON_CTL0_MSR                  0xF39
#define KNL_CHA26_PMON_CTL3_MSR                  0xF3C
#define KNL_CHA25_PMON_CTL0_MSR                  0xF2D
#define KNL_CHA25_PMON_CTL3_MSR                  0xF30
#define KNL_CHA24_PMON_CTL0_MSR                  0xF21
#define KNL_CHA24_PMON_CTL3_MSR                  0xF24
#define KNL_CHA23_PMON_CTL0_MSR                  0xF15
#define KNL_CHA23_PMON_CTL3_MSR                  0xF18
#define KNL_CHA22_PMON_CTL0_MSR                  0xF09
#define KNL_CHA22_PMON_CTL3_MSR                  0xF0C
#define KNL_CHA21_PMON_CTL0_MSR                  0xEFD
#define KNL_CHA21_PMON_CTL3_MSR                  0xF00
#define KNL_CHA20_PMON_CTL0_MSR                  0xEF1
#define KNL_CHA20_PMON_CTL3_MSR                  0xEF4
#define KNL_CHA19_PMON_CTL0_MSR                  0xEE5
#define KNL_CHA19_PMON_CTL3_MSR                  0xEE8
#define KNL_CHA18_PMON_CTL0_MSR                  0xED9
#define KNL_CHA18_PMON_CTL3_MSR                  0xEDC
#define KNL_CHA17_PMON_CTL0_MSR                  0xECD
#define KNL_CHA17_PMON_CTL3_MSR                  0xED0
#define KNL_CHA16_PMON_CTL0_MSR                  0xEC1
#define KNL_CHA16_PMON_CTL3_MSR                  0xEC4
#define KNL_CHA15_PMON_CTL0_MSR                  0xEB5
#define KNL_CHA15_PMON_CTL3_MSR                  0xEB8
#define KNL_CHA14_PMON_CTL0_MSR                  0xEA9
#define KNL_CHA14_PMON_CTL3_MSR                  0xEAC
#define KNL_CHA13_PMON_CTL0_MSR                  0xE9D
#define KNL_CHA13_PMON_CTL3_MSR                  0xEA0
#define KNL_CHA12_PMON_CTL0_MSR                  0xE91
#define KNL_CHA12_PMON_CTL3_MSR                  0xE94
#define KNL_CHA11_PMON_CTL0_MSR                  0xE85
#define KNL_CHA11_PMON_CTL3_MSR                  0xE88
#define KNL_CHA10_PMON_CTL0_MSR                  0xE79
#define KNL_CHA10_PMON_CTL3_MSR                  0xE7C
#define KNL_CHA9_PMON_CTL0_MSR                   0xE6D
#define KNL_CHA9_PMON_CTL3_MSR                   0xE70
#define KNL_CHA8_PMON_CTL0_MSR                   0xE61
#define KNL_CHA8_PMON_CTL3_MSR                   0xE64
#define KNL_CHA7_PMON_CTL0_MSR                   0xE55
#define KNL_CHA7_PMON_CTL3_MSR                   0xE58
#define KNL_CHA6_PMON_CTL0_MSR                   0xE49
#define KNL_CHA6_PMON_CTL3_MSR                   0xE4C
#define KNL_CHA5_PMON_CTL0_MSR                   0xE3D
#define KNL_CHA5_PMON_CTL3_MSR                   0xE40
#define KNL_CHA4_PMON_CTL0_MSR                   0xE31
#define KNL_CHA4_PMON_CTL3_MSR                   0xE34
#define KNL_CHA3_PMON_CTL0_MSR                   0xE25
#define KNL_CHA3_PMON_CTL3_MSR                   0xE28
#define KNL_CHA2_PMON_CTL0_MSR                   0xE19
#define KNL_CHA2_PMON_CTL3_MSR                   0xE1C
#define KNL_CHA1_PMON_CTL0_MSR                   0xE0D
#define KNL_CHA1_PMON_CTL3_MSR                   0xE10
#define KNL_CHA0_PMON_CTL0_MSR                   0xE01
#define KNL_CHA0_PMON_CTL3_MSR                   0xE04

#define IS_THIS_KNL_BOX_CTL_MSR(x)              ( x == KNL_CHA0_BOX_CTL_MSR  || x == KNL_CHA1_BOX_CTL_MSR  || x == KNL_CHA2_BOX_CTL_MSR  || x == KNL_CHA3_BOX_CTL_MSR \
                                               || x == KNL_CHA4_BOX_CTL_MSR  || x == KNL_CHA5_BOX_CTL_MSR  || x == KNL_CHA6_BOX_CTL_MSR  || x == KNL_CHA7_BOX_CTL_MSR \
                                               || x == KNL_CHA8_BOX_CTL_MSR  || x == KNL_CHA9_BOX_CTL_MSR  || x == KNL_CHA10_BOX_CTL_MSR || x == KNL_CHA11_BOX_CTL_MSR \
                                               || x == KNL_CHA12_BOX_CTL_MSR || x == KNL_CHA13_BOX_CTL_MSR || x == KNL_CHA14_BOX_CTL_MSR || x == KNL_CHA15_BOX_CTL_MSR \
                                               || x == KNL_CHA16_BOX_CTL_MSR || x == KNL_CHA17_BOX_CTL_MSR || x == KNL_CHA18_BOX_CTL_MSR || x == KNL_CHA19_BOX_CTL_MSR \
                                               || x == KNL_CHA20_BOX_CTL_MSR || x == KNL_CHA21_BOX_CTL_MSR || x == KNL_CHA22_BOX_CTL_MSR || x == KNL_CHA23_BOX_CTL_MSR \
                                               || x == KNL_CHA24_BOX_CTL_MSR || x == KNL_CHA25_BOX_CTL_MSR || x == KNL_CHA26_BOX_CTL_MSR || x == KNL_CHA27_BOX_CTL_MSR \
                                               || x == KNL_CHA28_BOX_CTL_MSR || x == KNL_CHA29_BOX_CTL_MSR || x == KNL_CHA30_BOX_CTL_MSR || x == KNL_CHA31_BOX_CTL_MSR \
                                               || x == KNL_CHA32_BOX_CTL_MSR || x == KNL_CHA33_BOX_CTL_MSR || x == KNL_CHA34_BOX_CTL_MSR || x == KNL_CHA35_BOX_CTL_MSR \
                                               || x == KNL_CHA36_BOX_CTL_MSR || x == KNL_CHA37_BOX_CTL_MSR)

#define  DISABLE_KNL_CHA_COUNTERS                0x100
#define  ENABLE_KNL_CHA_COUNTERS                 0x400000

#define IS_THIS_KNL_EVSEL_PMON_CTL_MSR(x)              ( (x>= KNL_CHA0_PMON_CTL0_MSR  && x <= KNL_CHA0_PMON_CTL3_MSR)  || (x>= KNL_CHA1_PMON_CTL0_MSR && x <= KNL_CHA1_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA2_PMON_CTL0_MSR  && x <= KNL_CHA2_PMON_CTL3_MSR)  || (x>= KNL_CHA3_PMON_CTL0_MSR && x <= KNL_CHA3_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA4_PMON_CTL0_MSR  && x <= KNL_CHA4_PMON_CTL3_MSR)  || (x>= KNL_CHA5_PMON_CTL0_MSR && x <= KNL_CHA5_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA6_PMON_CTL0_MSR  && x <= KNL_CHA6_PMON_CTL3_MSR)  || (x>= KNL_CHA7_PMON_CTL0_MSR && x <= KNL_CHA7_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA8_PMON_CTL0_MSR  && x <= KNL_CHA8_PMON_CTL3_MSR)  || (x>= KNL_CHA9_PMON_CTL0_MSR && x <= KNL_CHA9_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA10_PMON_CTL0_MSR && x <= KNL_CHA10_PMON_CTL3_MSR) || (x>= KNL_CHA11_PMON_CTL0_MSR && x <= KNL_CHA11_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA12_PMON_CTL0_MSR && x <= KNL_CHA12_PMON_CTL3_MSR) || (x>= KNL_CHA13_PMON_CTL0_MSR && x <= KNL_CHA13_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA14_PMON_CTL0_MSR && x <= KNL_CHA14_PMON_CTL3_MSR) || (x>= KNL_CHA15_PMON_CTL0_MSR && x <= KNL_CHA15_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA16_PMON_CTL0_MSR && x <= KNL_CHA16_PMON_CTL3_MSR) || (x>= KNL_CHA17_PMON_CTL0_MSR && x <= KNL_CHA17_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA18_PMON_CTL0_MSR && x <= KNL_CHA18_PMON_CTL3_MSR) || (x>= KNL_CHA19_PMON_CTL0_MSR && x <= KNL_CHA19_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA20_PMON_CTL0_MSR && x <= KNL_CHA20_PMON_CTL3_MSR) || (x>= KNL_CHA21_PMON_CTL0_MSR && x <= KNL_CHA21_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA22_PMON_CTL0_MSR && x <= KNL_CHA22_PMON_CTL3_MSR) || (x>= KNL_CHA23_PMON_CTL0_MSR && x <= KNL_CHA23_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA24_PMON_CTL0_MSR && x <= KNL_CHA24_PMON_CTL3_MSR) || (x>= KNL_CHA25_PMON_CTL0_MSR && x <= KNL_CHA25_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA26_PMON_CTL0_MSR && x <= KNL_CHA26_PMON_CTL3_MSR) || (x>= KNL_CHA27_PMON_CTL0_MSR && x <= KNL_CHA27_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA28_PMON_CTL0_MSR && x <= KNL_CHA28_PMON_CTL3_MSR) || (x>= KNL_CHA29_PMON_CTL0_MSR && x <= KNL_CHA29_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA30_PMON_CTL0_MSR && x <= KNL_CHA30_PMON_CTL3_MSR) || (x>= KNL_CHA31_PMON_CTL0_MSR && x <= KNL_CHA31_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA32_PMON_CTL0_MSR && x <= KNL_CHA32_PMON_CTL3_MSR) || (x>= KNL_CHA33_PMON_CTL0_MSR && x <= KNL_CHA33_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA34_PMON_CTL0_MSR && x <= KNL_CHA34_PMON_CTL3_MSR) || (x>= KNL_CHA35_PMON_CTL0_MSR && x <= KNL_CHA35_PMON_CTL3_MSR) \
                                                      || (x>= KNL_CHA36_PMON_CTL0_MSR && x <= KNL_CHA36_PMON_CTL3_MSR) || (x>= KNL_CHA37_PMON_CTL0_MSR && x <= KNL_CHA37_PMON_CTL3_MSR) )

extern  DISPATCH_NODE                            knl_cha_dispatch;
#define KNL_UBOX_GLOBAL_CONTROL_MSR              0x700

#endif

