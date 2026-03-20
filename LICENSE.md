<center>

# These are full license terms for
# <mark>STM32-Sidewalk-SDK</mark>

Copyright &copy;2025 STMicroelectronics

All rights reserved
    
</center>

## <mark>__OVERVIEW__</mark>
<div>

This Software Bill Of Material (SBOM) lists the software components of this
software package, including the copyright owner and license terms for each
component.

The full text of these licenses are below the SBOM.

__SOFTWARE BILL OF MATERIALS__

| Component                                                                                  | Copyright                                   | License
|----------                                                                                  |----------                                   |--------
| CMSIS                                                                                      | Arm Limited                                 | [Apache-2.0](https://opensource.org/licenses/Apache-2.0)
| STM32WBAxx CMSIS                                                                           | Arm Limited, STMicroelectronics             | [Apache-2.0](https://opensource.org/licenses/Apache-2.0)
| STM32WLxx CMSIS                                                                            | Arm Limited, STMicroelectronics             | [Apache-2.0](https://opensource.org/licenses/Apache-2.0)
| STM32WBAxx_HAL_Driver                                                                      | STMicroelectronics                          | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| STM32WLxx_HAL_Driver                                                                       | STMicroelectronics                          | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| BSP STM32WBAxx_Nucleo                                                                      | STMicroelectronics                          | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| BSP STM32WLxx_Nucleo                                                                       | STMicroelectronics                          | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| lib_gnss (Teseo GNSS Middleware)                                                           | STMicroelectronics                          | [ANNEX 1](#collapse-section2)
| STM32_Cryptographic (Middleware)                                                           | STMicroelectronics                          | [SLA](#collapse-section1)
| FreeRTOS (Middleware)                                                                      | Amazon.com, Inc. or its affiliates          | [MIT](https://opensource.org/license/MIT)
| lbm_lib (LoRa Basics Modem Middleware)                                                     | Semtech Corporation, STMicroelectronics     | [ANNEX 3](#collapse-section4)
| littlesfs (littleFS File System Middleware)                                                | The littlefs authors, Arm Limited           | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| STM32_WPAN (Middleware including Bluetooth Low Energy, Linklayer)                          | STMicroelectronics, Synopsys, Inc.          | [SLA](#collapse-section1)                                                                  
| SubGHz_Phy (Middleware)                                                                    | Semtech, STMicroelectronics                 | [ANNEX 3](#collapse-section4)
| Utilities                                                                                  | STMicroelectronics                          | [BSD-3-Clause](https://opensource.org/licenses/BSD-3-Clause)
| Sidewalk Unified Demo Apps (_apps/common_)                                                 | Amazon.com, Inc. or its affiliates          | [ANNEX 2](#collapse-section3)
| Common STM32 Platform Code (_apps/st/common_)                                              | STMicroelectronics                          | [SLA](#collapse-section1)
| Common STM32WBAxx Platform Code (_apps/st/stm32wba_)                                       | STMicroelectronics                          | [SLA](#collapse-section1)
| Sidewalk Sample Apps for WBAxx (_apps/st/stm32wba/sid_xxx_)                                | STMicroelectronics                          | [SLA](#collapse-section1)
| Sidewalk Sample Apps for WBAxx (_apps/st/stm32wba/sid_xxx_)                                | Amazon.com, Inc. or its affiliates          | [ANNEX 2](#collapse-section3)
| Sidewalk Sample Apps for WBAxx (_apps/st/stm32wba/sid_xxx_)                                | Semtech                                     | [ANNEX 3](#collapse-section4)
| Sidewalk Platform Abstraction Layer (PAL):  Common STM32 PAL (_pal/st/common_)             | STMicroelectronics                          | [SLA](#collapse-section1)
| Sidewalk Platform Abstraction Layer (PAL):  STM32WBAxx PAL (_pal/st/stm32wba_)             | STMicroelectronics                          | [SLA](#collapse-section1)
| Semtech Common Interfaces (_platform/sid_mcu/semtech/hal/common_)                          | Amazon.com, Inc. or its affiliates          | [ANNEX 2](#collapse-section3)
| Semtech LR11xx Sidewalk Driver (_platform/sid_mcu/semtech/hal/lr11xx_)                     | STMicroelectronics                          | [SLA](#collapse-section1)
| Semtech LR11xx Sidewalk Driver (_platform/sid_mcu/semtech/hal/lr11xx_)                     | Amazon.com, Inc. or its affiliates          | [ANNEX 2](#collapse-section3)
| Semtech LR11xx Driver (_platform/sid_mcu/semtech/hal/lr11xx/semtech_)                      | Semtech Corporation, STMicroelectronics     | [ANNEX 3](#collapse-section4)
| Semtech SX126x Sidewalk Driver (_platform/sid_mcu/semtech/hal/sx126x_)                     | STMicroelectronics                          | [SLA](#collapse-section1)
| Semtech SX126x Sidewalk Driver (_platform/sid_mcu/semtech/hal/sx126x_)                     | Amazon.com, Inc. or its affiliates          | [ANNEX 2](#collapse-section3)
| Semtech SX126x Driver (_platform/sid_mcu/semtech/hal/sx126x/semtech_)                      | Semtech Corporation, STMicroelectronics     | [ANNEX 3](#collapse-section4)
| LoRa Basics Modem Sidewalk Adapter (_platform/sid_mcu/semtech/lbm_sidewalk_adapter_)       | STMicroelectronics                          | [SLA](#collapse-section1)
| Spirit2 (S2-LP) Radio Driver (_platform/sid_mcu/st/hal/s2-lp_)                             | STMicroelectronics                          | [SLA](#collapse-section1)
| STM32WBAxx Sidewalk HAL (_platform/sid_mcu/st/hal/stm32wba_)                               | STMicroelectronics                          | [SLA](#collapse-section1)
| STM32WLxx Radio App Driver for WBAxx (_platform/sid_mcu/st/hal/stm32wlxx_radio_app_)       | STMicroelectronics, Semtech Corporation     | [SLA](#collapse-section1)
| Teseo GNSS Receivers Driver (_platform/sid_mcu/st/hal/teseo_)                              | STMicroelectronics                          | [SLA](#collapse-section1)
| Sidewalk MCU SDK Library and Code (_sidewalk_sdk_prebuilt_)                                | Amazon.com, Inc. or its affiliates          | [ANNEX 2](#collapse-section3)
| nanopb (_sidewalk_sdk_prebuilt_)                                                           | Petteri Aimonen                             | [Zlib](https://opensource.org/license/Zlib)
| LK embedded kernel (_sidewalk_sdk_prebuilt_)                                               | Travis Geiselbrecht                         | [MIT](https://opensource.org/license/MIT)
| uthash (_sidewalk_sdk_prebuilt_)                                                           | Troy D. Hanson                              | [BSD-1-Clause](https://opensource.org/licenses/BSD-1-Clause)
| Tool to sign OTA images (_tools/firmware_signing_)                                         | STMicroelectronics                          | [SLA](#collapse-section1)
| Sidewalk Provision Tool (_tools/provision_)                                                | Amazon.com, Inc. or its affiliates          | [Apache-2.0](https://opensource.org/licenses/Apache-2.0)

__Notes:__ If the license is an open source license, then you can access the
terms at [www.opensource.org](https://opensource.org/). Otherwise, the full
license terms are below. If a component is not listed in the SBOM, then the SLA
shall apply unless other terms are clearly stated in the package.

</div>

<label id="collapse-section1" aria-hidden="true">__SLA – Software License Agreement__</label>
<div>

SLA0048 Rev4/March 2018

## Software license agreement

### __SOFTWARE PACKAGE LICENSE AGREEMENT__

BY INSTALLING COPYING, DOWNLOADING, ACCESSING OR OTHERWISE USING THIS SOFTWARE PACKAGE OR ANY
PART THEREOF (AND THE RELATED DOCUMENTATION) FROM STMICROELECTRONICS INTERNATIONAL N.V, SWISS
BRANCH AND/OR ITS AFFILIATED COMPANIES (STMICROELECTRONICS), THE RECIPIENT, ON BEHALF OF HIMSELF
OR HERSELF, OR ON BEHALF OF ANY ENTITY BY WHICH SUCH RECIPIENT IS EMPLOYED AND/OR ENGAGED
AGREES TO BE BOUND BY THIS SOFTWARE PACKAGE LICENSE AGREEMENT.

Under STMicroelectronics’ intellectual property rights and subject to applicable licensing terms for any third-party software
incorporated in this software package and applicable Open Source Terms (as defined here below), the redistribution,
reproduction and use in source and binary forms of the software package or any part thereof, with or without modification, are
permitted provided that the following conditions are met:

1. Redistribution of source code (modified or not) must retain any copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form, except as embedded into microcontroller or microprocessor device manufactured by or for
STMicroelectronics or a software update for such device, must reproduce the above copyright notice, this list of conditions
and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of STMicroelectronics nor the names of other contributors to this software package may be used to
endorse or promote products derived from this software package or part thereof without specific written permission.

4. This software package or any part thereof, including modifications and/or derivative works of this software package, must
be used and execute solely and exclusively on or in combination with a microcontroller or a microprocessor devices
manufactured by or for STMicroelectronics.

5. No use, reproduction or redistribution of this software package partially or totally may be done in any manner that would
subject this software package to any Open Source Terms (as defined below).

6. Some portion of the software package may contain software subject to Open Source Terms (as defined below) applicable
for each such portion (“Open Source Software”), as further specified in the software package. Such Open Source Software
is supplied under the applicable Open Source Terms and is not subject to the terms and conditions of license hereunder.
“Open Source Terms” shall mean any open source license which requires as part of distribution of software that the source
code of such software is distributed therewith or otherwise made available, or open source license that substantially
complies with the Open Source definition specified at www.opensource.org and any other comparable open source license
such as for example GNU General Public License (GPL), Eclipse Public License (EPL), Apache Software License, BSD
license and MIT license.

7. This software package may also include third party software as expressly specified in the software package subject to
specific license terms from such third parties. Such third party software is supplied under such specific license terms and is
not subject to the terms and conditions of license hereunder. By installing copying, downloading, accessing or otherwise
using this software package, the recipient agrees to be bound by such license terms with regard to such third party
software.

8. STMicroelectronics has no obligation to provide any maintenance, support or updates for the software package.

9. The software package is and will remain the exclusive property of STMicroelectronics and its licensors. The recipient will
not take any action that jeopardizes STMicroelectronics and its licensors' proprietary rights or acquire any rights in the
software package, except the limited rights specified hereunder.

10. The recipient shall comply with all applicable laws and regulations affecting the use of the software package or any part
thereof including any applicable export control law or regulation.

11. Redistribution and use of this software package partially or any part thereof other than as permitted under this license is
void and will automatically terminate your rights under this license.

THIS SOFTWARE PACKAGE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY
INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE PACKAGE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

EXCEPT AS EXPRESSLY PERMITTED HEREUNDER AND SUBJECT TO THE APPLICABLE LICENSING TERMS FOR ANY
THIRD-PARTY SOFTWARE INCORPORATED IN THE SOFTWARE PACKAGE AND OPEN SOURCE TERMS AS
APPLICABLE, NO LICENSE OR OTHER RIGHTS, WHETHER EXPRESS OR IMPLIED, ARE GRANTED UNDER ANY
PATENT OR OTHER INTELLECTUAL PROPERTY RIGHTS OF STMICROELECTRONICS OR ANY THIRD PARTY.

</div>


<label id="collapse-section2" aria-hidden="true">__ANNEX 1 - SLA0055__</label>
<div>

SLA0055 Rev4/May 2017

Software license agreement

SOFTWARE LICENSE AGREEMENT ("Agreement")
The Licensed Software as defined below may contain various software that are subject to different
license agreements. The terms and conditions of those license agreements are available below, as well
as in the header files and documentation file accompanying this delivery.
(i) SUBJECT TO THE PROVISIONS HERE BELOW FOR THE LICENSED SOFTWARE PROVIDED IN
SOURCE AND IN OBJECT CODE : THE TERMS OF ST SOFTWARE LICENSE AGREEMENT
(REPRODUCED BELOW) SHALL APPLY;
(ii) FOR THE LICENSED SOFTWARE OR ANY PART THEREOF WHICH CONTAINS OPEN SOURCE
SOFTWARE: THE APPLICABLE OPEN SOURCE TERMS SHALL APPLY. OPEN SOURCE TERMS
MEANS ANY OPEN SOURCE LICENSE THAT COMPLIES WITH THE OPEN SOURCE DEFINITION
SPECIFIED AT WWW.OPENSOURCE.ORG AND ANY OTHER COMPARABLE OPEN SOURCE
LICENSE SUCH AS FOR EXAMPLE GNU GENERAL PUBLIC LICENSE (GPL), ECLIPSE PUBLIC
LICENSE (EPL), APACHE SOFTWARE LICENSE, BSD LICENSE AND MIT LICENSE. SUCH OPEN
SOURCE SOFTWARE IS NOT SUBJECT TO THE TERMS OF THIS AGREEMENT TO THE EXTENT
THE TERMS OF THIS AGREEMENT ARE IN CONFLICT WITH SUCH APPLICABLE OPEN SOURCE
TERMS. EXCEPT FOR OPEN SOURCE SOFTWARE, YOU HAVE NO RIGHTS UNDER THIS
AGREEMENT TO, AND MAY NOT UNDER ANY CIRCUMSTANCES USE THE SOFTWARE OR ANY
PARTS THEREOF TO MAKE THEM SUBJECT TO ANY OPEN SOURCE TERMS. THESE ACTIONS
INCLUDE BUT ARE NOT LIMITED TO COMBINING THE SOFTWARE BY MEANS OF
INCORPORATION OR LINKING OR OTHERWISE;
(iii) THE LICENSED SOFTWARE OR ANY PARTS THEREOF MAY (I) REQUIRE LICENSES FROM
THIRD PARTIES CLAIMING INTELLECTUAL PROPERTY RIGHTS COVERING USE OR
IMPLEMENTATION OF THE LICENSED SOFTWARE OR (II) BE BASED ON INDUSTRY
RECOGNIZED STANDARDS OR SOFTWARE PROGRAMS PUBLISHED BY INDUSTRY
RECOGNIZED STANDARDS BODIES AND CERTAIN THIRD PARTIES MAY CLAIM TO OWN
INTELLECTUAL PROPERTY RIGHTS THAT COVER IMPLEMENTATION OR USE OF THOSE
STANDARDS. YOU AGREE THAT YOU ARE RESPONSIBLE FOR OBTAINING ANY SUCH LICENSE
WHICH MAY BE NEEDED, AND NO SUCH LICENSE IS PROVIDED BY ST OR ITS AFFILIATES TO
YOU .SUCH THIRD PARTY INTELLECTUAL PROPERTY RIGHTS ARE NOT SUBJECT TO THE
TERMS OF THIS AGREEMENT TO THE EXTENT THE TERMS OF THIS AGREEMENT ARE IN
CONFLICT WITH SUCH APPLICABLE THIRD PARTY INTELLECTUAL PROPERTY RIGHTS.
BY CLICKING ON THE “I ACCEPT” BUTTON BELOW OR BY INSTALLING, COPYING,
DOWNLOADING OR OTHERWISE USING THE SOFTWARE IN THIS DELIVERY, YOU
ACKNOWLEDGE THAT YOU HAVE READ THE VARIOUS LICENSE AGREEMENTS APPLICABLE
TO EACH SOFTWARE IN THIS DELIVERY AND YOU AGREE TO BE BOUND BY THE TERMS OF
THOSE LICENSES. IF YOU DO NOT AGREE WITH ANY CONDITION OF THOSE LICENSES, DO
NOT INSTALL, DOWNLOAD, ACCESS OR USE THE SOFTWARE IN THIS DELIVERY.


ST SOFTWARE LICENSE
By using this Licensed Software, You are agreeing to be bound by the terms and conditions of this
Agreement. Do not use the Licensed Software until You have read and agreed to the following terms
and conditions and with the other terms and conditions that may apply according with (ii) and (iii) here
above. The use of the Licensed Software implies automatically the acceptance of the following terms
and conditions.


DEFINITIONS
Affiliates: means any corporation, partnership, or other entity that, directly or indirectly, owns, is owned
by, or is under common ownership with ST, for so long as such ownership exists. For the purposes of
the foregoing, "own," "owned," or "ownership" shall mean ownership of more than fifty percent (50%) of
the stock or other equity interests entitled to vote for the election of directors or an equivalent governing
body.

Compiled Code: means a machine-executable code in binary format.

IP Rights: means all patents, patent applications, including with respect to patents, any patent rights
granted upon any reissue, division, continuation or continuation-in-part applications now or hereafter
filed, utility models issued or pending, registered and unregistered design rights, copyrights (including
the copyright on software in any code), semiconductor mask works, trade secrets, know-how, and other
similar statutory intellectual property or industrial rights, as well as applications for any such rights.

Feedback: means any recommendations, suggestions, comments and corrections, including but not
limited to code enhancement, code modifications or bug fixes, related to the Licensed Software and any
elements and parts thereof.

Licensed Field: means all markets and applications worldwide, excluding: (i) life supporting devices or
systems, (ii) automotive safety, nuclear, military and aerospace markets and applications, (iii) opensource applications in case the Licensed Software is not already contaminated by ST.

Licensed Software: means the enclosed SOFTWARE/FIRMWARE, EXAMPLES, PROJECT
TEMPLATE and all the related documentation and design tools licensed and delivered in the form of
object and/or source code as the case may be.
Product: means Your and Your end-users’ product or system, and all the related documentation, that
includes or incorporates the Licensed Software in Compiled Code and the ST Device and provided
further that such Licensed Software or derivative works of the Licensed Software execute solely and
exclusively on ST Device.
ST Device: means the combination of: a) one ST microcontroller and b) one ST integrated circuit
chosen by You provided that a) and b) are manufactured and sold by or for ST.


LICENSE
STMicroelectronics International NV a Dutch company with headquarter at 39, Chemin du Champ des
filles, 1228 Plan-les-Oates Geneva (“ST”) grants to You under IP Rights owned by ST and its Affiliates
or under which ST and its Affiliates has the right to grant a license a non-exclusive, worldwide, nontransferable (whether by assignment or otherwise unless expressly authorized by ST) non sublicensable, revocable, royalty-free limited license to use the Licensed Software to:
(i) make copies, prepare derivative works of the source code version of the Licensed Software for the
sole and exclusive purpose of developing versions of such Licensed Software only for use within the
Product exclusively in Compiled Code;
(ii) make copies, prepare derivative works of the object code versions of the Licensed Software for the
sole purpose of designing, developing and manufacturing the Products;
(iii) make copies, prepare derivative works of the documentation part of the Licensed Software
(including non-confidential comments from source code files if applicable), for the sole purpose of
providing documentation for the Product and its usage;
(iv) make, have made, use, sell, offer to sell, import and export or otherwise distribute Products also
through multiple tiers.


OWNERSHIP AND COPYRIGHT
Title to the Licensed Software, related documentation and all copies thereof remain with ST and/or its
licensors. You may not remove the copyrights notices from the Licensed Software and to any copies of 
the Licensed Software. You agree to prevent any unauthorized copying of the Licensed Software and
related documentation. You grants to ST and its Affiliates a non-exclusive, worldwide, perpetual,
irrevocable, royalty free, fully paid up, sub-licensable and transferable license to use, copy, modify and
distribute any Feedback You may have.
You agree that no press releases or announcements or any marketing, advertising or other promotional
materials related to this Agreement or referencing or implying ST or its trade names, trademarks, or
service marks can be released without ST prior written approval.


RESTRICTIONS
Unless otherwise explicitly stated in this Agreement, You may not sell, assign, sublicense, lease, rent or
otherwise distribute the Licensed Software for commercial purposes, in whole or in part. You
acknowledge and agree that any use, adaptation, translation or transcription of the Licensed Software or
any portion or derivative thereof, for use with (i) product that does not include ST Device and/or (ii) with
device, having similar functionalities to ST Devices, manufactured by or for an entity other than ST, is a
material breach of this Agreement and requires a separate license from ST. No source code relating to
and/or based upon Licensed Software is to be made available or sub-licensed by You unless expressly
permitted under the Section “License”. You acknowledge and agree that the protection of the source
code of the Licensed Software warrants the imposition of reasonable security precautions. In the event
ST demonstrates to You a reasonable belief that the source code of the Licensed Software has been
used or distributed in violation of this Agreement, ST may, by written notification, request certification as
to whether such unauthorized use or distribution has occurred. You shall cooperate and assist ST in its
determination of whether there has been unauthorized use or distribution of the source code of the
Licensed Software and will take appropriate steps to remedy any unauthorized use or distribution.


NO WARRANTY
TO THE EXTENT PERMITTED BY THE LAWS, THE LICENSED SOFTWARE IS PROVIDED “AS IS”
AND “WITH ALL FAULTS” WITHOUT WARRANTY OR REPRESENTATION OF ANY KIND
EXPRESSED OR IMPLIED AND ST AND ITS LICENSORS EXPRESSLY DISCLAIM ALL
WARRANTIES, EXPRESSED, IMPLIED OR OTHERWISE, INCLUDING BUT NOT LIMITED TO ,
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT OF THIRD PARTIES INTELLECTUAL PROPERTY RIGHTS.
IN PARTICULAR: A) ST DOES NOT WARRANT THAT THE USE IN WHOLE OR IN PART OF THE
LICENSED SOFTWARE WILL BE INTERRUPTED OR ERROR FREE, WILL MEET YOUR
REQUIREMENTS, OR WILL OPERATE WITH THE COMBINATION OF HARDWARE AND
SOFTWARE SELECTED BY YOU. YOU ARE RESPONSIBLE FOR DETERMINING WHETHER THE
LICENSED SOFTWARE WILL BE SUITABLE FOR YOUR INTENDED USE OR APPLICATION OR
WILL ACHIEVE YOUR INTENDED RESULTS. ST WILL NOT BE LIABLE TO YOU AND/OR TO ANY
THIRD PARTY FOR THE DERIVATIVE WORKS OF THE LICENSED SOFTWARE DEVELOPED BY
YOU. ST HAS NOT AUTHORIZED ANYONE TO MAKE ANY REPRESENTATION OR WARRANTY
FOR THE LICENSED SOFTWARE, AND ANY TECHNICAL, APPLICATIONS OR DESIGN
INFORMATION OR ADVICE, QUALITY CHARACTERIZATION, RELIABILITY DATA OR OTHER
SERVICES PROVIDED BY ST SHALL NOT CONSTITUTE ANY REPRESENTATION OR WARRANTY
BY ST OR ALTER THIS DISCLAIMER OR WARRANTY, AND IN NO ADDITIONAL OBLIGATIONS OR
LIABILITIES SHALL ARISE FROM ST’S PROVIDING SUCH INFORMATION OR SERVICES. ST
DOES NOT ASSUME OR AUTHORIZE ANY OTHER PERSON TO ASSUME FOR IT ANY OTHER
LIABILITY IN CONNECTION WITH ITS LICENSED SOFTWARE.
B) NOTHING CONTAINED IN THIS AGREEMENT WILL BE CONSTRUED AS:
(i) A WARRANTY OR REPRESENTATION BY ST TO MAINTAIN PRODUCTION OF ANY ST DEVICE
OR OTHER HARDWARE OR SOFTWARE WITH WHICH THE LICENSED SOFTWARE MAY BE
USED OR TO OTHERWISE MAINTAIN OR SUPPORT THE LICENSED SOFTWARE IN ANY
MANNER; AND (ii) A COMMITMENT FROM ST AND/OR ITS LICENSORS TO BRING OR PROSECUTE ACTIONS OR
SUITS AGAINST THIRD PARTIES FOR INFRINGEMENT OF ANY OF THE RIGHTS LICENSED
HEREBY, OR CONFERRING ANY RIGHTS TO BRING OR PROSECUTE ACTIONS OR SUITS
AGAINST THIRD PARTIES FOR INFRINGEMENT. HOWEVER, ST HAS THE RIGHT TO TERMINATE
THIS AGREEMENT IMMEDIATELY UPON RECEIVING NOTICE OF ANY CLAIM, SUIT OR
PROCEEDING THAT ALLEGES THAT THE LICENSED SOFTWARE OR YOUR USE OR
DISTRIBUTION OF THE LICENSED SOFTWARE INFRINGES ANY THIRD PARTY INTELLECTUAL
PROPERTY RIGHTS.
ALL OTHER WARRANTIES, CONDITIONS OR OTHER TERMS IMPLIED BY LAW ARE EXCLUDED
TO THE FULLEST EXTENT PERMITTED BY LAW.
LIMITATION OF LIABILITIES AND INDEMNIFICATION
TO THE EXTENT PERMITTED BY THE LAW IN NO EVENT ST , ITS AFFILIATES, ITS LICENSORS
AND SUBCONTRACTORS INCLUDING THE OFFICERS, DIRECTORS, EMPLOYEES,
SHAREHOLDERS OR AGENTS OF ANY OF THEM (“ST PARTIES”) SHALL BE LIABLE TO YOU OR
TO ANY THIRD PARTY THAT YOU MAY BIND TO THIS AGREEMENT FOR ANY INDIRECT,
SPECIAL, CONSEQUENTIAL, INCIDENTAL, PUNITIVE DAMAGES OR OTHER DAMAGES
(INCLUDING BUT NOT LIMITED TO, THE COST OF LABOUR, RE-QUALIFICATION, DELAY, LOSS
OF PROFITS, LOSS OF REVENUES, LOSS OF DATA, COSTS OF PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES OR THE LIKE) WHETHER BASED ON CONTRACT, TORT,
OR ANY OTHER LEGAL THEORY, RELATING TO OR IN CONNECTION WITH THE LICENSED
SOFTWARE, THE DOCUMENTATION OR THIS AGREEMENT, EVEN IF ST PARTIES HAVE BEEN
ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
IN NO EVENT SHALL ST’S LIABILITY TO YOU OR ANY THIRD PARTY THAT YOU MAY BIND TO
THIS AGREEMENT, UNDER THIS AGREEMENT (INCLUDING BUT NOT LIMITED TO CLAIMS
ARISING FROM OR IN CONNECTION WITH THE LICENSED SOFTWARE AND/OR ANY PART
THEREOF AND THE USE OF THEM) FOR ANY CAUSE OF ACTION EXCEED IN THE AGGREGATE
100 US$. THIS SECTION DOES NOT APPLY TO THE EXTENT PROHIBITED BY LAW.
YOU AGREE TO INDEMNIFY ST PARTIES FOR ANY DAMAGE OR LOSS THAT ANY AND ALL OF
THE ST PARTIES MAY SUFFER BECAUSE OF ANY THIRD PARTIES CLAIM THAT MAY ARISE
FROM OR THAT IS IN CONNECTION WITH THE USE YOU HAVE MADE OF THE LICENSED
SOFTWARE.


LIMITATION OF LIABILITIES AND INDEMNIFICATION
TO THE EXTENT PERMITTED BY THE LAW IN NO EVENT ST , ITS AFFILIATES, ITS LICENSORS
AND SUBCONTRACTORS INCLUDING THE OFFICERS, DIRECTORS, EMPLOYEES,
SHAREHOLDERS OR AGENTS OF ANY OF THEM (“ST PARTIES”) SHALL BE LIABLE TO YOU OR
TO ANY THIRD PARTY THAT YOU MAY BIND TO THIS AGREEMENT FOR ANY INDIRECT,
SPECIAL, CONSEQUENTIAL, INCIDENTAL, PUNITIVE DAMAGES OR OTHER DAMAGES
(INCLUDING BUT NOT LIMITED TO, THE COST OF LABOUR, RE-QUALIFICATION, DELAY, LOSS
OF PROFITS, LOSS OF REVENUES, LOSS OF DATA, COSTS OF PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES OR THE LIKE) WHETHER BASED ON CONTRACT, TORT,
OR ANY OTHER LEGAL THEORY, RELATING TO OR IN CONNECTION WITH THE LICENSED
SOFTWARE, THE DOCUMENTATION OR THIS AGREEMENT, EVEN IF ST PARTIES HAVE BEEN
ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
IN NO EVENT SHALL ST’S LIABILITY TO YOU OR ANY THIRD PARTY THAT YOU MAY BIND TO
THIS AGREEMENT, UNDER THIS AGREEMENT (INCLUDING BUT NOT LIMITED TO CLAIMS
ARISING FROM OR IN CONNECTION WITH THE LICENSED SOFTWARE AND/OR ANY PART
THEREOF AND THE USE OF THEM) FOR ANY CAUSE OF ACTION EXCEED IN THE AGGREGATE
100 US$. THIS SECTION DOES NOT APPLY TO THE EXTENT PROHIBITED BY LAW.
YOU AGREE TO INDEMNIFY ST PARTIES FOR ANY DAMAGE OR LOSS THAT ANY AND ALL OF
THE ST PARTIES MAY SUFFER BECAUSE OF ANY THIRD PARTIES CLAIM THAT MAY ARISE
FROM OR THAT IS IN CONNECTION WITH THE USE YOU HAVE MADE OF THE LICENSED
SOFTWARE.


TERMINATION
This Agreement shall be effective as of the moment in which You accept this terms and conditions and
shall terminate 10 (ten) years thereafter. ST may terminate this Agreement: a) at any time if You are in
material breach of any of its terms and conditions and You have failed to cure such breach within 30
(thirty) days upon occurrence of such breach b) upon 60 days prior notice to You. Upon termination,
You will immediately destroy or return all copies of the Licensed Software and related documentation to
ST. After termination, You will be entitled to use the Licensed Software used into Products that include
ST Device manufactured by or for ST, purchased by You before the date of the termination.


COMPLIANCE
You agree not to use the Licensed Software in violation of any applicable law, statute, ordinance or
other regulation or any obligation and to procure any information that may be needed for such a
purpose. You agree to comply with all applicable laws and regulations affecting the use of the Licensed
Software. Specifically but without limiting the generality of the foregoing, You acknowledges that the
Licensed Software and/or some of its possible usage could be subject to export controls restrictions
and/or personal data law protection and You agree to enquire about any such possible export controls
restrictions or data protection restriction law and regulation with the competent authorities, comply with
any applicable export control and personal data protection law or regulation including but not limited to
the European export and personal data regulations and US similar regulations, and to obtain any 
necessary export license, authorizations or other documentations prior to the exportation or reexportation of the Licensed Software and/or prior to apply the usage concerned. You shall promptly
procure and shall deliver to ST any declaration and/or certificate that ST may reasonably require in
order to be compliant with any export control law and regulation. It is also understood that ST is entitled
to refuse the delivery of the Licensed Software in case ST should reasonably suspect that such delivery
or the usage by You is in breach of any applicable export control law and regulation or any other
applicable law.


APPLICABLE LAW AND JURISDICTION
This Agreement shall be construed, governed and enforced in accordance with the law of Switzerland
without regards to its conflict of law provisions. For sake of clarity the United Nations Convention on
Contracts for the International Sale of Goods (Vienna, 1980) (CISG) shall not apply.
Any dispute arising out of or in connection with this Agreement shall be exclusively settled by the Courts
of Geneva, Switzerland.. Notwithstanding the aforesaid, nothing in this Agreement shall prevent ST from
seeking any interim or final injunctive or equitable relief by a court of competent jurisdiction.


SEVERABILITY
If any provision of this agreement or any part thereof is or becomes, at any time or for any reason,
unenforceable or invalid, no the remaining part of any other provision of this agreement shall be affected
thereby, and the remaining provisions of this agreement shall continue with the same force and effect as
if such unenforceable or invalid provisions or parts thereof had not been inserted in this Agreement.


ENTIRE AGREEMENT
The terms and conditions contained herein constitute the entire agreement between You and ST and
shall supersede all previous communications either oral or written, between You and ST with respect to
the subject matter hereof. No oral explanation or oral information by You, ST and its Affiliates hereto
shall alter the meaning or interpretation of this Agreement. This Agreement may not be modified or any
right of You or ST waived, except by means of an amendment which expressly references this
Agreement and is duly executed by duly authorised representatives of You, ST and its Affiliates.


WAIVER
The waiver by either party of any breach of any provisions of this Agreement shall not operate or be
construed as a waiver of any other or a subsequent breach of the same or a different provision.


ASSIGNMENT
This Agreement may not be assigned by You, nor any of Your rights or obligations hereunder, to any
third party without prior written consent of ST (which shall not be unreasonably withheld). In the event
that this Agreement is assigned effectively to a third party, this Agreement shall bind upon successors
and assigns of the parties hereto. Notwithstanding the foregoing, ST may assign this Agreement as well
as its rights and obligations in whole or in part to any of its Affiliates as well as to any person or entity, in
connection with any transfer of any ST business or part of business to which this Agreement pertains,
by merger, consolidation, reorganization or otherwise.


RELATIONSHIP OF THE PARTIES
Nothing in this Agreement shall create, or be deemed to create, a partnership or the relationship of
principal and agent or employer and employee between the Parties. Neither Party has the authority or
power to bind, to contract in the name of or to create a liability for the other in any way or for any
purpose.

</div>


<label id="collapse-section3" aria-hidden="true">__ANNEX 2 - AMAZON SOFTWARE LICENSE TERMS__</label>
<div>

Amazon Sidewalk Content License

©2023 Amazon.com, Inc. or its affiliates (collectively, “Amazon”). All Rights Reserved.

These materials are licensed to you as "AWS Content" under the AWS Intellectual Property License (https://aws.amazon.com/legal/aws-ip-license-terms/) (the “IP License”) and made available to you solely in connection with the developer program for Amazon Sidewalk and related services (such AWS Content, “Amazon Sidewalk Content”). 

Capitalized terms not defined in this file have the meanings given to them in the AWS Customer Agreement (https://aws.amazon.com/agreement/) or the Service Terms (https://aws.amazon.com/service-terms/), including any documentation referenced or incorporated therein (collectively, the “Agreement”).

In addition to the terms and conditions of the IP License, the following terms and conditions ("Additional Terms," and collectively with the IP License, the "Terms") apply to the Amazon Sidewalk Content. In the event of a conflict between the IP License and these Additional Terms, the IP License will control.

You may only use the Amazon Sidewalk Content to (a) evaluate, develop, test, and troubleshoot the operation and compatibility of the Amazon Sidewalk Content with your AS Devices; and (b) integrate the Amazon Sidewalk Content into your AS Devices that access Amazon Sidewalk. If any of the Amazon Sidewalk Content incorporated in your AS Device were provided to you in source code form, such Amazon Sidewalk Content (and modifications thereof) may be distributed solely in binary form as incorporated in your AS Devices. You may not distribute any of your AS Devices that incorporate or were developed using the Amazon Sidewalk Content unless you submit your AS Device to us for approval and we specifically approve the distribution in writing.  You may not: (i) use Amazon Sidewalk Content with any software or other materials that are subject to licenses or restrictions (e.g., open source software licenses) that, when combined with Amazon Sidewalk Content, would require you or us to disclose, license, distribute, or otherwise make all or any part of such Amazon Sidewalk Content available to anyone; or (ii) remove, modify, or obscure any copyright, patent, trademark, or other proprietary or attribution notices on or in any Amazon Sidewalk Content.

Notwithstanding the foregoing, if you are a system-on-chip manufacturer, you may distribute the Amazon Sidewalk Content, including any source code (subject to the limitations herein) made available by Amazon, to AS Device manufacturers for the purpose of such AS Device manufacturers integrating the Amazon Sidewalk Content in their AS Devices and their associated applications and services; provided that you also make available this LICENSE.TXT file to such device manufacturer together with the Amazon Sidewalk Content. For the purpose of the Terms, such Amazon Sidewalk Content will be deemed to have been made available to the device manufacturers from Amazon.

WITH THE EXCEPTION OF FILES IDENTIFIED AS A "MODIFIABLE FILE" WITHIN THE FILE HEADER, YOU MAY NOT MODIFY ANY SOURCE CODE IN ANY FILE IN THIS PACKAGE. Your modifications to the Amazon Sidewalk Content are “Modified Content”. You may reproduce and distribute copies of the Modified Content thereof in any medium, with or without modifications, and in source or object form, provided that you meet the following conditions:

* You must give any other recipients of the Modified Content a copy of these Terms; and

* You must cause any modified files to carry prominent notices stating that you changed the files; and

* You must retain, in the source form of any Modified Content that you distribute, all copyright, patent, trademark, and attribution notices from the source form of the Amazon Sidewalk Content, excluding those notices that do not pertain to any part of the Modified Content; and

* If the Amazon Sidewalk Content includes a "NOTICE" text file as part of its distribution, then any Modified Content that you distribute must include a readable copy of the attribution notices contained within such NOTICE file, excluding those notices that do not pertain to any part of the Modified Content, in at least one of the following places: within a NOTICE text file distributed as part of the Modified Content; within the source form or documentation, if provided along with the Modified Content; or, within a display generated by the Modified Content, if and wherever such third-party notices normally appear. The contents of the NOTICE file are for informational purposes only and do not modify the Terms. You may add your own attribution notices within Modified Content that you distribute, alongside or as an addendum to the NOTICE text from the Amazon Sidewalk Content, provided that such additional attribution notices cannot be construed as modifying these Terms.

You may add your own copyright statement to your modifications and may provide additional or different license terms and conditions for use, reproduction, or distribution of your modifications, or for any such Modified Content as a whole, provided your use, reproduction, and distribution of the Amazon Sidewalk Content otherwise complies with the conditions stated in these Terms.

Our licensors may enforce the Terms against you with respect to their software and other materials included in the Amazon Sidewalk Content, and our licensors are third-party beneficiaries of these Terms solely for that purpose.

Amazon Sidewalk Content may include and/or be dependent upon certain third-party libraries or other software packages ("External Dependencies").  If you do not agree with every term in the license file associated with the External Dependencies, you should not use these materials.

THE AMAZON SIDEWALK CONTENT AND THE EXTERNAL DEPENDENCIES ARE PROVIDED BY AMAZON AND AMAZON’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. AMAZON DOES NOT PROMISE THAT THE EXTERNAL DEPENDENCIES AND THEIR APPLICABLE TERMS ARE COMPLETE, ACCURATE, OR UP-TO-DATE, AND AMAZON WILL HAVE NO LIABILITY FOR ANY OMISSIONS OR INACCURACIES. YOU SHOULD CONSULT THE DOWNLOAD SITES FOR THE EXTERNAL DEPENDENCIES FOR THE MOST COMPLETE AND UP-TO-DATE LICENSING INFORMATION. YOUR USE OF THE AMAZON SIDEWALK CONTENT AND THE EXTERNAL DEPENDENCIES IS AT YOUR SOLE RISK. IN NO EVENT WILL AMAZON BE LIABLE FOR ANY DAMAGES, INCLUDING WITHOUT LIMITATION ANY DIRECT, INDIRECT, CONSEQUENTIAL, SPECIAL, INCIDENTAL, OR PUNITIVE DAMAGES (INCLUDING FOR ANY LOSS OF GOODWILL, BUSINESS INTERRUPTION, LOST PROFITS OR DATA, OR COMPUTER FAILURE OR MALFUNCTION) ARISING FROM OR RELATING TO THE AMAZON SIDEWALK CONTENT OR EXTERNAL DEPENDENCIES, HOWEVER CAUSED AND REGARDLESS OF THE THEORY OF LIABILITY, EVEN IF AMAZON HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES. THESE LIMITATIONS AND DISCLAIMERS APPLY EXCEPT TO THE EXTENT PROHIBITED BY APPLICABLE LAW.

You will defend, indemnify, and hold harmless Amazon, its affiliates, and licensors against any and all claims, losses, and damages arising out of or relating to your distribution of the Amazon Sidewalk Content or External Dependencies in breach of the Terms.

If you do not agree to the Terms, you may not use any file in this package.

</div>


<label id="collapse-section4" aria-hidden="true">__ANNEX 3 - The Clear BSD License__</label>
<div>

The Clear BSD License
Copyright Semtech Corporation 2023. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

* Neither the name of the Semtech corporation nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

</div>
