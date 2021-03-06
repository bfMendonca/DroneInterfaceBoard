/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.0-dev */

#ifndef PB_MPU6500_PB_H_INCLUDED
#define PB_MPU6500_PB_H_INCLUDED
#include <pb.h>

#include "vector3.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _MPU6500Readings {
    Vector3 accelerometer;
    Vector3 gyroscope;
    float temperature;
    uint64_t timestamp;
/* @@protoc_insertion_point(struct:MPU6500Readings) */
} MPU6500Readings;

/* Default values for struct fields */

/* Initializer values for message structs */
#define MPU6500Readings_init_default             {Vector3_init_default, Vector3_init_default, 0, 0}
#define MPU6500Readings_init_zero                {Vector3_init_zero, Vector3_init_zero, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define MPU6500Readings_accelerometer_tag        1
#define MPU6500Readings_gyroscope_tag            2
#define MPU6500Readings_temperature_tag          3
#define MPU6500Readings_timestamp_tag            4

/* Struct field encoding specification for nanopb */
extern const pb_field_t MPU6500Readings_fields[5];

/* Maximum encoded size of messages (where known) */
#define MPU6500Readings_size                     50

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define MPU6500_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
