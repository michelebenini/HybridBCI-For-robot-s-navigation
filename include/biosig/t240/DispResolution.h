/*
 * Generated by asn1c-0.9.21 (http://lionet.info/asn1c)
 * From ASN.1 module "FEF-IntermediateDraft"
 * 	found in "../annexb-snacc-122001.asn1"
 */

#ifndef	_DispResolution_H_
#define	_DispResolution_H_


#include <asn_application.h>

/* Including external dependencies */
#include <INTEGER.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* DispResolution */
typedef struct DispResolution {
	INTEGER_t	 prepoint;
	INTEGER_t	 postpoint;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DispResolution_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DispResolution;

#ifdef __cplusplus
}
#endif

#endif	/* _DispResolution_H_ */
