/*
 * Generated by asn1c-0.9.21 (http://lionet.info/asn1c)
 * From ASN.1 module "FEF-IntermediateDraft"
 * 	found in "../annexb-snacc-122001.asn1"
 */

#include <asn_internal.h>

#include "PatientDemographicsSection.h"

static asn_TYPE_member_t asn_MBR_diagnosticcodes_26[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_ExtNomenRef,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		""
		},
};
static ber_tlv_tag_t asn_DEF_diagnosticcodes_tags_26[] = {
	(ASN_TAG_CLASS_APPLICATION | (2492 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_diagnosticcodes_specs_26 = {
	sizeof(struct PatientDemographicsSection__diagnosticcodes),
	offsetof(struct PatientDemographicsSection__diagnosticcodes, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_diagnosticcodes_26 = {
	"diagnosticcodes",
	"diagnosticcodes",
	SEQUENCE_OF_free,
	SEQUENCE_OF_print,
	SEQUENCE_OF_constraint,
	SEQUENCE_OF_decode_ber,
	SEQUENCE_OF_encode_der,
	SEQUENCE_OF_decode_xer,
	SEQUENCE_OF_encode_xer,
	0, 0,	/* No PER support, use "-gen-PER" to enable */
	0,	/* Use generic outmost tag fetcher */
	asn_DEF_diagnosticcodes_tags_26,
	sizeof(asn_DEF_diagnosticcodes_tags_26)
		/sizeof(asn_DEF_diagnosticcodes_tags_26[0]) - 1, /* 1 */
	asn_DEF_diagnosticcodes_tags_26,	/* Same as above */
	sizeof(asn_DEF_diagnosticcodes_tags_26)
		/sizeof(asn_DEF_diagnosticcodes_tags_26[0]), /* 2 */
	0,	/* No PER visible constraints */
	asn_MBR_diagnosticcodes_26,
	1,	/* Single element */
	&asn_SPC_diagnosticcodes_specs_26	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_procedurecodes_32[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_ExtNomenRef,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		""
		},
};
static ber_tlv_tag_t asn_DEF_procedurecodes_tags_32[] = {
	(ASN_TAG_CLASS_APPLICATION | (2493 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_procedurecodes_specs_32 = {
	sizeof(struct PatientDemographicsSection__procedurecodes),
	offsetof(struct PatientDemographicsSection__procedurecodes, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_procedurecodes_32 = {
	"procedurecodes",
	"procedurecodes",
	SEQUENCE_OF_free,
	SEQUENCE_OF_print,
	SEQUENCE_OF_constraint,
	SEQUENCE_OF_decode_ber,
	SEQUENCE_OF_encode_der,
	SEQUENCE_OF_decode_xer,
	SEQUENCE_OF_encode_xer,
	0, 0,	/* No PER support, use "-gen-PER" to enable */
	0,	/* Use generic outmost tag fetcher */
	asn_DEF_procedurecodes_tags_32,
	sizeof(asn_DEF_procedurecodes_tags_32)
		/sizeof(asn_DEF_procedurecodes_tags_32[0]) - 1, /* 1 */
	asn_DEF_procedurecodes_tags_32,	/* Same as above */
	sizeof(asn_DEF_procedurecodes_tags_32)
		/sizeof(asn_DEF_procedurecodes_tags_32[0]), /* 2 */
	0,	/* No PER visible constraints */
	asn_MBR_procedurecodes_32,
	1,	/* Single element */
	&asn_SPC_procedurecodes_specs_32	/* Additional specs */
};

static asn_TYPE_member_t asn_MBR_PatientDemographicsSection_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct PatientDemographicsSection, handle),
		(ASN_TAG_CLASS_APPLICATION | (2337 << 2)),
		0,
		&asn_DEF_Handle,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"handle"
		},
	{ ATF_POINTER, 31, offsetof(struct PatientDemographicsSection, patientid),
		(ASN_TAG_CLASS_APPLICATION | (2394 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FEFString,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"patientid"
		},
	{ ATF_POINTER, 30, offsetof(struct PatientDemographicsSection, ungroupedname),
		(ASN_TAG_CLASS_APPLICATION | (6001 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FEFString,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"ungroupedname"
		},
	{ ATF_POINTER, 29, offsetof(struct PatientDemographicsSection, characternamegroup),
		(ASN_TAG_CLASS_APPLICATION | (6002 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PersonNameGroup,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"characternamegroup"
		},
	{ ATF_POINTER, 28, offsetof(struct PatientDemographicsSection, ideographicnamegroup),
		(ASN_TAG_CLASS_APPLICATION | (6003 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PersonNameGroup,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"ideographicnamegroup"
		},
	{ ATF_POINTER, 27, offsetof(struct PatientDemographicsSection, phoneticnamegroup),
		(ASN_TAG_CLASS_APPLICATION | (6004 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PersonNameGroup,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"phoneticnamegroup"
		},
	{ ATF_POINTER, 26, offsetof(struct PatientDemographicsSection, birthname),
		(ASN_TAG_CLASS_APPLICATION | (2398 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FEFString,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"birthname"
		},
	{ ATF_POINTER, 25, offsetof(struct PatientDemographicsSection, sex),
		(ASN_TAG_CLASS_APPLICATION | (2401 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PatientSex,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"sex"
		},
	{ ATF_POINTER, 24, offsetof(struct PatientDemographicsSection, race),
		(ASN_TAG_CLASS_APPLICATION | (2526 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PatientRace,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"race"
		},
	{ ATF_POINTER, 23, offsetof(struct PatientDemographicsSection, patienttype),
		(ASN_TAG_CLASS_APPLICATION | (2402 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PatientType,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"patienttype"
		},
	{ ATF_POINTER, 22, offsetof(struct PatientDemographicsSection, dateofbirth),
		(ASN_TAG_CLASS_APPLICATION | (2392 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_AbsoluteTime,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"dateofbirth"
		},
	{ ATF_POINTER, 21, offsetof(struct PatientDemographicsSection, patientgeninfo),
		(ASN_TAG_CLASS_APPLICATION | (2393 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FEFString,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"patientgeninfo"
		},
	{ ATF_POINTER, 20, offsetof(struct PatientDemographicsSection, patientage),
		(ASN_TAG_CLASS_APPLICATION | (2520 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PatMeasure,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"patientage"
		},
	{ ATF_POINTER, 19, offsetof(struct PatientDemographicsSection, gestationalage),
		(ASN_TAG_CLASS_APPLICATION | (2521 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PatMeasure,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"gestationalage"
		},
	{ ATF_POINTER, 18, offsetof(struct PatientDemographicsSection, patientheight),
		(ASN_TAG_CLASS_APPLICATION | (2524 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PatMeasure,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"patientheight"
		},
	{ ATF_POINTER, 17, offsetof(struct PatientDemographicsSection, patientweight),
		(ASN_TAG_CLASS_APPLICATION | (2527 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PatMeasure,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"patientweight"
		},
	{ ATF_POINTER, 16, offsetof(struct PatientDemographicsSection, patientbirthlength),
		(ASN_TAG_CLASS_APPLICATION | (2522 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PatMeasure,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"patientbirthlength"
		},
	{ ATF_POINTER, 15, offsetof(struct PatientDemographicsSection, patientbirthweight),
		(ASN_TAG_CLASS_APPLICATION | (2523 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PatMeasure,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"patientbirthweight"
		},
	{ ATF_POINTER, 14, offsetof(struct PatientDemographicsSection, motherpatientid),
		(ASN_TAG_CLASS_APPLICATION | (2504 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FEFString,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"motherpatientid"
		},
	{ ATF_POINTER, 13, offsetof(struct PatientDemographicsSection, mothername),
		(ASN_TAG_CLASS_APPLICATION | (2525 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PersonName,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"mothername"
		},
	{ ATF_POINTER, 12, offsetof(struct PatientDemographicsSection, patientheadcircumference),
		(ASN_TAG_CLASS_APPLICATION | (2490 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PatMeasure,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"patientheadcircumference"
		},
	{ ATF_POINTER, 11, offsetof(struct PatientDemographicsSection, patientbsa),
		(ASN_TAG_CLASS_APPLICATION | (2390 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PatMeasure,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"patientbsa"
		},
	{ ATF_POINTER, 10, offsetof(struct PatientDemographicsSection, bedid),
		(ASN_TAG_CLASS_APPLICATION | (2501 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FEFString,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"bedid"
		},
	{ ATF_POINTER, 9, offsetof(struct PatientDemographicsSection, diagnosticinfo),
		(ASN_TAG_CLASS_APPLICATION | (2496 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FEFString,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"diagnosticinfo"
		},
	{ ATF_POINTER, 8, offsetof(struct PatientDemographicsSection, diagnosticcodes),
		(ASN_TAG_CLASS_APPLICATION | (2492 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_diagnosticcodes_26,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"diagnosticcodes"
		},
	{ ATF_POINTER, 7, offsetof(struct PatientDemographicsSection, admittingphysician),
		(ASN_TAG_CLASS_APPLICATION | (2515 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_HandleRef,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"admittingphysician"
		},
	{ ATF_POINTER, 6, offsetof(struct PatientDemographicsSection, attendingphysician),
		(ASN_TAG_CLASS_APPLICATION | (2516 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_HandleRef,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"attendingphysician"
		},
	{ ATF_POINTER, 5, offsetof(struct PatientDemographicsSection, dateofprocedure),
		(ASN_TAG_CLASS_APPLICATION | (2518 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_AbsoluteTime,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"dateofprocedure"
		},
	{ ATF_POINTER, 4, offsetof(struct PatientDemographicsSection, proceduredescription),
		(ASN_TAG_CLASS_APPLICATION | (2495 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_FEFString,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"proceduredescription"
		},
	{ ATF_POINTER, 3, offsetof(struct PatientDemographicsSection, procedurecodes),
		(ASN_TAG_CLASS_APPLICATION | (2493 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_procedurecodes_32,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"procedurecodes"
		},
	{ ATF_POINTER, 2, offsetof(struct PatientDemographicsSection, anaesthetist),
		(ASN_TAG_CLASS_APPLICATION | (2479 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_HandleRef,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"anaesthetist"
		},
	{ ATF_POINTER, 1, offsetof(struct PatientDemographicsSection, surgeon),
		(ASN_TAG_CLASS_APPLICATION | (2532 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_HandleRef,
		0,	/* Defer constraints checking to the member type */
		0,	/* PER is not compiled, use -gen-PER */
		0,
		"surgeon"
		},
};
static ber_tlv_tag_t asn_DEF_PatientDemographicsSection_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_TYPE_tag2member_t asn_MAP_PatientDemographicsSection_tag2el_1[] = {
    { (ASN_TAG_CLASS_APPLICATION | (2337 << 2)), 0, 0, 0 }, /* handle at 340 */
    { (ASN_TAG_CLASS_APPLICATION | (2390 << 2)), 21, 0, 0 }, /* patientbsa at 383 */
    { (ASN_TAG_CLASS_APPLICATION | (2392 << 2)), 10, 0, 0 }, /* dateofbirth at 357 */
    { (ASN_TAG_CLASS_APPLICATION | (2393 << 2)), 11, 0, 0 }, /* patientgeninfo at 359 */
    { (ASN_TAG_CLASS_APPLICATION | (2394 << 2)), 1, 0, 0 }, /* patientid at 343 */
    { (ASN_TAG_CLASS_APPLICATION | (2398 << 2)), 6, 0, 0 }, /* birthname at 348 */
    { (ASN_TAG_CLASS_APPLICATION | (2401 << 2)), 7, 0, 0 }, /* sex at 351 */
    { (ASN_TAG_CLASS_APPLICATION | (2402 << 2)), 9, 0, 0 }, /* patienttype at 355 */
    { (ASN_TAG_CLASS_APPLICATION | (2479 << 2)), 30, 0, 0 }, /* anaesthetist at 412 */
    { (ASN_TAG_CLASS_APPLICATION | (2490 << 2)), 20, 0, 0 }, /* patientheadcircumference at 381 */
    { (ASN_TAG_CLASS_APPLICATION | (2492 << 2)), 24, 0, 0 }, /* diagnosticcodes at 392 */
    { (ASN_TAG_CLASS_APPLICATION | (2493 << 2)), 29, 0, 0 }, /* procedurecodes at 408 */
    { (ASN_TAG_CLASS_APPLICATION | (2495 << 2)), 28, 0, 0 }, /* proceduredescription at 403 */
    { (ASN_TAG_CLASS_APPLICATION | (2496 << 2)), 23, 0, 0 }, /* diagnosticinfo at 388 */
    { (ASN_TAG_CLASS_APPLICATION | (2501 << 2)), 22, 0, 0 }, /* bedid at 386 */
    { (ASN_TAG_CLASS_APPLICATION | (2504 << 2)), 18, 0, 0 }, /* motherpatientid at 376 */
    { (ASN_TAG_CLASS_APPLICATION | (2515 << 2)), 25, 0, 0 }, /* admittingphysician at 394 */
    { (ASN_TAG_CLASS_APPLICATION | (2516 << 2)), 26, 0, 0 }, /* attendingphysician at 397 */
    { (ASN_TAG_CLASS_APPLICATION | (2518 << 2)), 27, 0, 0 }, /* dateofprocedure at 400 */
    { (ASN_TAG_CLASS_APPLICATION | (2520 << 2)), 12, 0, 0 }, /* patientage at 362 */
    { (ASN_TAG_CLASS_APPLICATION | (2521 << 2)), 13, 0, 0 }, /* gestationalage at 365 */
    { (ASN_TAG_CLASS_APPLICATION | (2522 << 2)), 16, 0, 0 }, /* patientbirthlength at 372 */
    { (ASN_TAG_CLASS_APPLICATION | (2523 << 2)), 17, 0, 0 }, /* patientbirthweight at 374 */
    { (ASN_TAG_CLASS_APPLICATION | (2524 << 2)), 14, 0, 0 }, /* patientheight at 368 */
    { (ASN_TAG_CLASS_APPLICATION | (2525 << 2)), 19, 0, 0 }, /* mothername at 378 */
    { (ASN_TAG_CLASS_APPLICATION | (2526 << 2)), 8, 0, 0 }, /* race at 353 */
    { (ASN_TAG_CLASS_APPLICATION | (2527 << 2)), 15, 0, 0 }, /* patientweight at 370 */
    { (ASN_TAG_CLASS_APPLICATION | (2532 << 2)), 31, 0, 0 }, /* surgeon at 415 */
    { (ASN_TAG_CLASS_APPLICATION | (6001 << 2)), 2, 0, 0 }, /* ungroupedname at 195 */
    { (ASN_TAG_CLASS_APPLICATION | (6002 << 2)), 3, 0, 0 }, /* characternamegroup at 196 */
    { (ASN_TAG_CLASS_APPLICATION | (6003 << 2)), 4, 0, 0 }, /* ideographicnamegroup at 197 */
    { (ASN_TAG_CLASS_APPLICATION | (6004 << 2)), 5, 0, 0 } /* phoneticnamegroup at 198 */
};
static asn_SEQUENCE_specifics_t asn_SPC_PatientDemographicsSection_specs_1 = {
	sizeof(struct PatientDemographicsSection),
	offsetof(struct PatientDemographicsSection, _asn_ctx),
	asn_MAP_PatientDemographicsSection_tag2el_1,
	32,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* Start extensions */
	-1	/* Stop extensions */
};
asn_TYPE_descriptor_t asn_DEF_PatientDemographicsSection = {
	"PatientDemographicsSection",
	"PatientDemographicsSection",
	SEQUENCE_free,
	SEQUENCE_print,
	SEQUENCE_constraint,
	SEQUENCE_decode_ber,
	SEQUENCE_encode_der,
	SEQUENCE_decode_xer,
	SEQUENCE_encode_xer,
	0, 0,	/* No PER support, use "-gen-PER" to enable */
	0,	/* Use generic outmost tag fetcher */
	asn_DEF_PatientDemographicsSection_tags_1,
	sizeof(asn_DEF_PatientDemographicsSection_tags_1)
		/sizeof(asn_DEF_PatientDemographicsSection_tags_1[0]), /* 1 */
	asn_DEF_PatientDemographicsSection_tags_1,	/* Same as above */
	sizeof(asn_DEF_PatientDemographicsSection_tags_1)
		/sizeof(asn_DEF_PatientDemographicsSection_tags_1[0]), /* 1 */
	0,	/* No PER visible constraints */
	asn_MBR_PatientDemographicsSection_1,
	32,	/* Elements count */
	&asn_SPC_PatientDemographicsSection_specs_1	/* Additional specs */
};

