/*
    MS OS 2.0 USB descriptors support

    Copyright (C) 2022-2023 SudoMaker, Ltd.
    Author: Reimu NotMoe <reimu@sudomaker.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

    SudoMaker, Ltd. hereby grants you additional permissions under section 7 of
    the GNU Affero General Public License version 3 ("AGPLv3") in accordance
    with the following terms:

    1. You are permitted to modify and distribute this program or any
    covered work without being required to distribute the source code of your
    modifications, provided that the resulting executable (binary code) is
    exclusively designed and intended to run on (a) hardware devices
    manufactured by SudoMaker, Ltd., (b) hardware components manufactured by
    SudoMaker, Ltd. which are parts of a larger hardware device, or (c)
    hardware devices that are certified through the "Resonance Certification
    Program" by SudoMaker, Ltd.

    2. This additional permission is granted solely for the purpose of enabling
    the use of this program on the aforementioned hardware devices or
    components, and does not extend to any other use or distribution of this
    program.

    3. If you distribute or make available any executable (binary code) that
    was modified under the terms of this additional permission, you must state
    that it is distributed or made available solely for use on the
    aforementioned hardware devices or components.

    4. These additional permissions are granted to you in conjunction with the
    AGPLv3 and does not affect other rights or obligations under that license.

    By exercising any of the additional permissions granted here, you agree to
    be bound by the terms and conditions mentioned above.
*/

#ifndef	__LINUX_USB_MSOS20_H
#define	__LINUX_USB_MSOS20_H

#include "uapi/linux/usb/ch9.h"

/*
 * little endian PlatformCapablityUUID for MS OS 2.0
 * D8DD60DF-4589-4CC7-9CD2-659D9E648A9F
 * to identify Platform Device Capability descriptors as referring to MS OS 2.0
 *
 * the UUID above MUST be sent over the wire as the byte sequence:
 * {0x38, 0xB6, 0x08, 0x34, 0xA9, 0x09, 0xA0, 0x47, 0x8B, 0xFD, 0xA0, 0x76, 0x88, 0x15, 0xB6, 0x65}.
 */
#define MSOS20_UUID \
	UUID_INIT(0xdf60ddd8, 0x8945, 0xc74c, 0x9c, 0xd2, 0x65, 0x9d, 0x9e, 0x64, 0x8a, 0x9f)

/*
 * MS OS 2.0 Descriptor set information data
 *
 */
struct usb_msos20_desc_set_info_data {
	__le32	dwWindowsVersion;
	__le16	wMSOSDescriptorSetTotalLength;
	u8	bMS_VendorCode;
	u8	bAltEnumCode;
} __packed;

#define MSOS20_WINDOWS_VERSION_8_1		cpu_to_le32(0x06030000)

#define USB_MSOS20_DESC_SET_INFO_SIZE		8

#define MSOS20_DESCRIPTOR_INDEX			0x7
#define MSOS20_SET_ALT_ENUMERATION		0x8

#define MSOS20_SET_HEADER_DESCRIPTOR		0x00
#define MSOS20_SUBSET_HEADER_CONFIGURATION	0x01
#define MSOS20_SUBSET_HEADER_FUNCTION		0x02
#define MSOS20_FEATURE_COMPATBLE_ID		0x03
#define MSOS20_FEATURE_REG_PROPERTY		0x04
#define MSOS20_FEATURE_MIN_RESUME_TIME		0x05
#define MSOS20_FEATURE_MODEL_ID			0x06
#define MSOS20_FEATURE_CCGP_DEVICE		0x07
#define MSOS20_FEATURE_VENDOR_REVISION		0x08

#define MSOS20_REG_SZ				1
#define MSOS20_REG_EXPAND_SZ			2
#define MSOS20_REG_BINARY			3
#define MSOS20_REG_DWORD_LITTLE_ENDIAN		4
#define MSOS20_REG_DWORD_BIG_ENDIAN		5
#define MSOS20_REG_LINK				6
#define MSOS20_REG_MULTI_SZ			7


// Microsoft OS 2.0 descriptor set header
struct usb_msos20_desc_set_hdr_data {
	__le16	wLength;
	__le16	wDescriptorType;
	__le32	dwWindowsVersion;
	__le16	wTotalLength;
} __packed;

// Microsoft OS 2.0 configuration subset header
struct usb_msos20_cfg_subset_hdr_data {
	__le16	wLength;
	__le16	wDescriptorType;
	u8	bConfigurationValue;
	u8	bReserved;
	__le16	wTotalLength;
} __packed;

// Microsoft OS 2.0 function subset header
struct usb_msos20_func_subset_hdr_data {
	__le16	wLength;
	__le16	wDescriptorType;
	u8	bFirstInterface;
	u8	bReserved;
	__le16	wSubsetLength;
} __packed;

// Microsoft OS 2.0 compatible ID descriptor
struct usb_msos20_compat_id_desc_data {
	__le16	wLength;
	__le16	wDescriptorType;
	u8	CompatibleID[8];
	u8	SubCompatibleID[8];
} __packed;

// Microsoft OS 2.0 registry property descriptor
struct usb_msos20_reg_prop_desc_data {
	__le16	wLength;
	__le16	wDescriptorType;
} __packed;

// Microsoft OS 2.0 minimum USB recovery time descriptor
struct usb_msos20_min_usb_rec_time_desc_data {
	__le16	wLength;
	__le16	wDescriptorType;
	u8	bResumeRecoveryTime;
	u8	bResumeSignalingTime;
} __packed;

// Microsoft OS 2.0 model ID descriptor
struct usb_msos20_model_id_data {
	__le16	wLength;
	__le16	wDescriptorType;
	u8	ModelID[16];
} __packed;

// Microsoft OS 2.0 CCGP device descriptor
struct usb_msos20_ccgp_dev_data {
	__le16	wLength;
	__le16	wDescriptorType;
} __packed;

// Microsoft OS 2.0 vendor revision descriptor
struct usb_msos20_vendor_rev_data {
	__le16	wLength;
	__le16	wDescriptorType;
	__le16	VendorRevision;
} __packed;

#endif /* __LINUX_USB_MSOS20_H */
