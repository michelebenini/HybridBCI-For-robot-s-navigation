/*
 * Generated by asn1c-0.9.21 (http://lionet.info/asn1c)
 * From ASN.1 module "FEF-IntermediateDraft"
 * 	found in "../annexb-snacc-122001.asn1"
 */

#ifndef	_ManufacturerSpecificBinary_H_
#define	_ManufacturerSpecificBinary_H_


#include <asn_application.h>

/* Including external dependencies */
#include "PrivateCode.h"
#include <OCTET_STRING.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ManufacturerSpecificBinary */
typedef struct ManufacturerSpecificBinary {
	PrivateCode_t	 code;
	OCTET_STRING_t	 data;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ManufacturerSpecificBinary_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ManufacturerSpecificBinary;

#ifdef __cplusplus
}
#endif

#endif	/* _ManufacturerSpecificBinary_H_ */
