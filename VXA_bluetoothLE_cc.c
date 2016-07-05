/******************************************************************************/
/**
 * This software is copyrighted by Bosch Connected Devices and Solutions GmbH, 2015.
 * The use of this software is subject to the XDK SDK EULA
 */
/**
 *  @file        VXA_bluetooth_cc.c
 *
 * ****************************************************************************/

/* module includes ********************************************************** */

/* own header files */
#include "VXA_bluetoothLE_ih.h"
#include "VXA_virtualXdkEmbedded_ih.h"
#include "VXA_bluetoothLE_ch.h"

/* system header files */
#include <stdio.h>
#include <BCDS_Basics.h>
#include <math.h>

/* additional interface header files */
#include "PTD_portDriver_ph.h"
#include "PTD_portDriver_ih.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "BLE_stateHandler_ih.h"
#include "BLE_serialDriver_ih.h"
#include "attserver.h"
#include "BleGap.h"
#include "BleTypes.h"
#include "BCDS_Magnetometer.h"
#include "BCDS_Assert.h"

/* constant definitions ***************************************************** */

/** Accelerometer Sensor Service UUID: 5a 21 1d 40 71 66 11 e4 82 f8 08 00 20 0c 9a 66 */
const uint8_t acceleratorSensorServiceUUID[16] = { 0x66, 0x9A, 0x0C, 0x20, 0x00,
		0x08, 0xF8, 0x82, 0xE4, 0x11, 0x66, 0x71, 0x40, 0x1D, 0x21, 0x5A };

/** Gyro Sensor Service Sensor UUID: ac a9 6a 40 74 a4 11 e4 82 f8 08 00 20 0c 9a 66 */
const uint8_t gyroSensorServiceUUID[16] = { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08,
		0xF8, 0x82, 0xE4, 0x11, 0xA4, 0x74, 0x40, 0x6A, 0xA9, 0xAC };

/** Light Sensor Service UUID: 38eb02c0-7540-11e4-82f8-0800200c9a66*/
const uint8_t lightSensorServiceUUID[16] = { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08,
		0xF8, 0x82, 0xE4, 0x11, 0x40, 0x75, 0xC0, 0x02, 0xEB, 0x38 };

/** Noise Sensor Service UUID: 38eb02c0-7540-11e4-82f8-0800200c9a66*/
const uint8_t noiseSensorServiceUUID[16] = { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08,
		0xF8, 0x82, 0xE4, 0x11, 0x4C, 0x75, 0x30, 0x38, 0x03, 0x01 };

/** Magnetometer Sensor Service UUID: 651f4c00-7579-11e4-82f8-0800200c9a66 */
const uint8_t magnetometerSensorServiceUUID[16] = { 0x66, 0x9A, 0x0C, 0x20,
		0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0x79, 0x75, 0x00, 0x4C, 0x1F, 0x65 };

/** Environment Sensor Service UUID: 92dab060-7634-11e4-82f8-0800200c9a66 */
const uint8_t environmentSensorServiceUUID[16] = { 0x66, 0x9A, 0x0C, 0x20, 0x00,
		0x08, 0xF8, 0x82, 0xE4, 0x11, 0x34, 0x76, 0x60, 0xB0, 0xDA, 0x92 };

/** High Data Rate Service UUID: c2967210-7ba4-11e4-82f8-0800200c9a66 */
const uint8_t highDataRateServiceUUID[16] = { 0x66, 0x9A, 0x0C, 0x20, 0x00,
		0x08, 0xF8, 0x82, 0xE4, 0x11, 0xA4, 0x7B, 0x10, 0x72, 0x96, 0xC2 };

/** Control XDK Service UUID: 55b741d0-7ada-11e4-82f8-0800200c9a66 */
const uint8_t controlXdkServiceUUID[16] = { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08,
		0xF8, 0x82, 0xE4, 0x11, 0xDA, 0x7A, 0xD0, 0x41, 0xB7, 0x55 };

/* local variables ********************************************************** */

static xTaskHandle VXA_bleTask; /**<Task handle for the BLE application handler*/

static uint8_t peer; /** Connection handle to client */

static AttServiceAttribute acceleratorSensorService; /** Attribute handle for the accelerometer sensor service */

static AttServiceAttribute gyroSensorService; /** Attribute handle for the gyro sensor service */

static AttServiceAttribute lightSensorService; /** Attribute handle for the light sensor service */

static AttServiceAttribute noiseSensorService; /** Attribute handle for the noise sensor service */

static AttServiceAttribute magnetometerSensorService; /** Attribute handle for the magnetometer sensor service */

static AttServiceAttribute environmentSensorService; /** Attribute handle for the environment sensor service */

static AttServiceAttribute highDataRateService; /** Attribute handle for the hight data rate service */

static AttServiceAttribute controlXdkService; /** Attribute handle for the XDK control service */

/** Characteristic properties for the accelerometer sensor */
static characteristicProperty_t accelerometerCharacteristicProperties[3] = {
/* X-Axis characteristic properties */
/* Accelerometer X-Axis sensor value UUID: 5a211d41-7166-11e4-82f8-0800200c9a66 */
{ { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0x66, 0x71,
		0x41, 0x1D, 0x21, 0x5A }, { 0 }, { 0 }, { 0 } },
/* Y-Axis characteristic properties */
/* Accelerometer Y-Axis sensor value UUID: 5a211d42-7166-11e4-82f8-0800200c9a66 */
{ { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0x66, 0x71,
		0x42, 0x1D, 0x21, 0x5A }, { 0 }, { 0 }, { 0 } },
/* Z-Axis characteristic properties */
/* Accelerometer Z-Axis sensor value UUID: 5a211d43-7166-11e4-82f8-0800200c9a66 */
{ { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0x66, 0x71,
		0x43, 0x1D, 0x21, 0x5A }, { 0 }, { 0 }, { 0 } } };

/** Characteristic properties for the gyro sensor */
static characteristicProperty_t gyroCharacteristicProperties[3] = {
/* X-Axis characteristic properties */
/* Gyro X-Axis sensor value UUID: aca96a41-74a4-11e4-82f8-0800200c9a66 */
{ { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0xA4, 0x74,
		0x41, 0x6A, 0xA9, 0xAC }, { 0 }, { 0 }, { 0 } },
/* Y-Axis characteristic properties */
/* Gyro Y-Axis sensor value UUID: aca96a42-74a4-11e4-82f8-0800200c9a66 */
{ { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0xA4, 0x74,
		0x42, 0x6A, 0xA9, 0xAC }, { 0 }, { 0 }, { 0 } },
/* Z-Axis characteristic properties */
/* Gyro Z-Axis sensor value UUID: aca96a43-74a4-11e4-82f8-0800200c9a66 */
{ { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0xA4, 0x74,
		0x43, 0x6A, 0xA9, 0xAC }, { 0 }, { 0 }, { 0 } } };

/** Characteristic properties for the light sensor
 * UUID: 38eb02c1-7540-11e4-82f8-0800200c9a66
 */
static characteristicProperty_t lightSensorCharacteristicProperties = { { 0x66,
		0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0x40, 0x75, 0xC1,
		0x02, 0xEB, 0x38 }, { 0 }, { 0 }, { 0 } };

/** Characteristic properties for the noise sensor
 * UUID: 01033831-754c-11e4-82f8-0800200c9a66
 */
static characteristicProperty_t noiseSensorCharacteristicProperties = { { 0x66,
		0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0x4C, 0x75, 0x31,
		0x38, 0x03, 0x01 }, { 0 }, { 0 }, { 0 } };

/** Characteristic properties for the magnetometer sensor */
static characteristicProperty_t magnetometerSensorCharacteristicProperties[4] =
		{
		/* X-Axis characteristic properties */
		/* Magnetometer X-Axis sensor value UUID: 651f4c01-7579-11e4-82f8-0800200c9a66 */
		{ { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0x79,
				0x75, 0x01, 0x4C, 0x1F, 0x65 }, { 0 }, { 0 }, { 0 } },
		/* Y-Axis characteristic properties */
		/* Magnetometer Y-Axis sensor value UUID: 651f4c02-7579-11e4-82f8-0800200c9a66 */
		{ { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0x79,
				0x75, 0x02, 0x4C, 0x1F, 0x65 }, { 0 }, { 0 }, { 0 } },
		/* Z-Axis characteristic properties */
		/* Magnetometer Z-Axis sensor value UUID: 651f4c03-7579-11e4-82f8-0800200c9a66 */
		{ { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0x79,
				0x75, 0x03, 0x4C, 0x1F, 0x65 }, { 0 }, { 0 }, { 0 } },
		/* Magnetometer resistance characteristic properties */
		/* Magnetometer sensor resistance value UUID: 651f4c04-7579-11e4-82f8-0800200c9a66 */
		{ { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0x79,
				0x75, 0x04, 0x4C, 0x1F, 0x65 }, { 0 }, { 0 }, { 0 } }, };

/** Characteristic properties for the environment sensor pressure value
 * UUID: 92dab061-7634-11e4-82f8-0800200c9a66
 */
static characteristicProperty_t environmentSensorPressureCharacteristicProperties =
		{ { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0x34,
				0x76, 0x61, 0xB0, 0xDA, 0x92 }, { 0 }, { 0 }, { 0 } };

/** Characteristic properties for the environment sensor temperature value
 * UUID: 92dab062-7634-11e4-82f8-0800200c9a66
 */
static characteristicProperty_t environmentSensorTemperatureCharacteristicProperties =
		{ { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0x34,
				0x76, 0x62, 0xB0, 0xDA, 0x92 }, { 0 }, { 0 }, { 0 } };

/** Characteristic properties for the environment sensor humidity value
 * UUID: 92dab063-7634-11e4-82f8-0800200c9a66
 */
static characteristicProperty_t environmentSensorHumidityCharacteristicProperties =
		{ { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0x34,
				0x76, 0x63, 0xB0, 0xDA, 0x92 }, { 0 }, { 0 }, { 0 } };

/** Characteristic property for the starting sensor sampling characteristic
 * UUID: 55b741d1-7ada-11e4-82f8-0800200c9a66
 */
static characteristicProperty_t startSensorSamplingAndNotificationsCharacteristicProperties =
		{ { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0xDA,
				0x7A, 0xD1, 0x41, 0xB7, 0x55 }, { 0 }, { 0 }, { 0 } };

/** Characteristic property for the setting the sensor sampling rate characteristic
 * UUID: 55b741d2-7ada-11e4-82f8-0800200c9a66
 */
static characteristicProperty_t setSamplingRateCharacteristicProperties = { {
		0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0xDA, 0x7A,
		0xD2, 0x41, 0xB7, 0x55 }, { 0 }, { 0 }, { 0 } };

/** Characteristic property for reset the XDK characteristic
 * UUID: 55b741d3-7ada-11e4-82f8-0800200c9a66
 */
static characteristicProperty_t resetXdkCharacteristicProperties = { { 0x66,
		0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0xDA, 0x7A, 0xD3,
		0x41, 0xB7, 0x55 }, { 0 }, { 0 }, { 0 } };

/** Characteristic property for getting the firmware version sampling characteristic
 * UUID: 55b741d4-7ada-11e4-82f8-0800200c9a66
 */
static characteristicProperty_t getXdkFirmwareVersionCharacteristicProperties =
		{ { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0xDA,
				0x7A, 0xD4, 0x41, 0xB7, 0x55 }, { 0 }, { 0 }, { 0 } };

/** Characteristic properties for the high rate data service */
static characteristicProperty_t highDataRateCharacteristicProperties[] = {
/* high priority characteristic properties */
/* high priority value UUID: c2967211-7ba4-11e4-82f8-0800200c9a66 */
{ { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0xA4, 0x7B,
		0x11, 0x72, 0x96, 0xC2 }, { 0 }, { 0 }, { 0 } },
/*low priority characteristic properties */
/*low priority value UUID: c2967212-7ba4-11e4-82f8-0800200c9a66 */
{ { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0xF8, 0x82, 0xE4, 0x11, 0xA4, 0x7B,
		0x12, 0x72, 0x96, 0xC2 }, { 0 }, { 0 }, { 0 } } };

/** Container for the accelerometer sensor values send via BLE */
static uint8_t acceleratorSensorCharacteristicValues[3][2];

/** Container for the gyro sensor values send via BLE */
static uint8_t gyroSensorCharacteristicValues[3][2];

/** Container for the light sensor values send via BLE */
static uint8_t lightSensorCharacteristicValue[20];

/** Container for the noise sensor values send via BLE */
static uint8_t noiseSensorCharacteristicValue[1];

/** Container for the magnetometer sensor values send via BLE */
static uint8_t magnetometerSensorCharacteristicValues[4][2];

/** Container for the pressure sensor values send via BLE */
static uint8_t environmentSensorPressureCharacteristicValue[4];

/** Container for the temperature sensor values send via BLE */
static uint8_t environmentSensorTemperatureCharacteristicValue[4];

/** Container for the humidity sensor values send via BLE */
static uint8_t environmentSensorHumidityCharacteristicValue[4];

/** Container for the high data rate values send via BLE */
static uint8_t highDataRateCharacteristicValue[2][20];

/** Container for the start sensor sampling message received via BLE */
static uint8_t startSensorSamplingCharacteristicValue[1];

/** Container for the sampling time message received via BLE */
static uint8_t setSensorSamplingTimerCharacteristicValue[4];

/** Container for the reset XDK message received via BLE */
static uint8_t resetXdkCharacteristicValue[1];

/**
 * Firmware version information
 * Format: {year, month, day, build-number}
 */
static const uint8_t firmwareVersionCharacteristicValue[] = { 15, 01, 13, 01 };

/** Helper variable for toggling sample timer on and off */
static uint8_t toggleTimerOnOff = 0;

/* global variables ********************************************************* */

/* inline functions ********************************************************* */

/* local functions ********************************************************** */

static void vxaBleHandler(void *pParameters) {
	(void) pParameters; /* to quiet warnings */

	/* return variable for stack receive status from base band */
	BLE_return_t bleTrsprtRetVal;

	/* return variable for BLE state handler */
	uint8_t bleStateHandleRetVal;

	for (;;) {
		/* Notify the BLE Stack that some data have been received from the Base band(Chip) or Host */
		bleTrsprtRetVal = BLE_hciReceiveData();

		/* This function is used to run the BLE stack and register a BLE device with the specified role */
		bleStateHandleRetVal = BLE_coreStateMachine();

		/* future use */
		UNUSED_PARAMETER(bleTrsprtRetVal);
		UNUSED_PARAMETER(bleStateHandleRetVal);
	}
}

static void vxaBleServiceRegister(void) {
	/* flag for service registry return */
	BleStatus serviceRegistryStatus;

	/* Accelerator sensor Service Register */
	serviceRegistryStatus = vxaBleRegisterAccelerometer();
	assert(serviceRegistryStatus != BLESTATUS_FAILED);

	/* Gyro sensor Service Register */
	serviceRegistryStatus = vxaBleRegisterGyro();
	assert(serviceRegistryStatus != BLESTATUS_FAILED);

	/* Light sensor Service Register */
	serviceRegistryStatus = vxaBleRegisterLightSensor();
	assert(serviceRegistryStatus != BLESTATUS_FAILED);

	/* Noise sensor Service Register */
	serviceRegistryStatus = vxaBleRegisterNoiseSensor();
	assert(serviceRegistryStatus != BLESTATUS_FAILED);

	/* Magnetometer sensor Service Register */
	serviceRegistryStatus = vxaBleRegisterMagnetometerSensor();
	assert(serviceRegistryStatus != BLESTATUS_FAILED);

	/* Environment sensor Service Register */
	serviceRegistryStatus = vxaBleRegisterEnvironmnetSensor();
	assert(serviceRegistryStatus != BLESTATUS_FAILED);

	/* High data rate Service Register */
	serviceRegistryStatus = vxaBleRegisterHighDataRateService();
	assert(serviceRegistryStatus != BLESTATUS_FAILED);

	/* Control XDK Service Register */
	serviceRegistryStatus = vxaBleRegisterXdkControl();
	assert(serviceRegistryStatus != BLESTATUS_FAILED);

	(void) serviceRegistryStatus;
}

static BleStatus vxaBleRegisterAccelerometer(void) {
	ATT_SERVER_SecureDatabaseAccess();

	if (ATT_SERVER_RegisterServiceAttribute(
	ATTPDU_SIZEOF_128_BIT_UUID, (uint8_t *) acceleratorSensorServiceUUID,
			acceleratorSensorServiceCallback,
			&(acceleratorSensorService)) == BLESTATUS_FAILED) {
		ATT_SERVER_ReleaseDatabaseAccess();
		return BLESTATUS_FAILED;
	}

	ATT_SERVER_ReleaseDatabaseAccess();

	for (uint8_t i = 0; i < 3; i++) {
		ATT_SERVER_SecureDatabaseAccess();

		accelerometerCharacteristicProperties[i].uuidType.size =
				ATT_UUID_SIZE_128;
		accelerometerCharacteristicProperties[i].uuidType.value.uuid128 =
				(uint8_t *) &accelerometerCharacteristicProperties[i].characteristicUUID;

		acceleratorSensorCharacteristicValues[i][0] = 0;

		if (ATT_SERVER_AddCharacteristic(
		ATTPROPERTY_READ | ATTPROPERTY_NOTIFY,
				(Att16BitCharacteristicAttribute*) &accelerometerCharacteristicProperties[i].characteristic,
				&accelerometerCharacteristicProperties[i].uuidType,
				ATT_PERMISSIONS_ALLACCESS, 2,
				(uint8_t *) &highDataRateCharacteristicValue[i][0], 0, 0,
				&acceleratorSensorService,
				&accelerometerCharacteristicProperties[i].characteristicAttribute) == BLESTATUS_FAILED) {
			ATT_SERVER_ReleaseDatabaseAccess();
			return BLESTATUS_FAILED;
		}

		ATT_SERVER_ReleaseDatabaseAccess();
	}
	return BLESTATUS_SUCCESS;
}

static BleStatus vxaBleRegisterGyro(void) {
	ATT_SERVER_SecureDatabaseAccess();

	if (ATT_SERVER_RegisterServiceAttribute(
	ATTPDU_SIZEOF_128_BIT_UUID, (uint8_t *) gyroSensorServiceUUID,
			gyroSensorServiceCallback, &(gyroSensorService)) == BLESTATUS_FAILED) {
		ATT_SERVER_ReleaseDatabaseAccess();
		return BLESTATUS_FAILED;
	}

	ATT_SERVER_ReleaseDatabaseAccess();

	for (uint8_t i = 0; i < 3; i++) {
		ATT_SERVER_SecureDatabaseAccess();

		gyroCharacteristicProperties[i].uuidType.size = ATT_UUID_SIZE_128;
		gyroCharacteristicProperties[i].uuidType.value.uuid128 =
				(uint8_t *) &gyroCharacteristicProperties[i].characteristicUUID;

		gyroSensorCharacteristicValues[i][0] = 0;

		if (ATT_SERVER_AddCharacteristic(
		ATTPROPERTY_READ | ATTPROPERTY_NOTIFY,
				(Att16BitCharacteristicAttribute*) &gyroCharacteristicProperties[i].characteristic,
				&gyroCharacteristicProperties[i].uuidType,
				ATT_PERMISSIONS_ALLACCESS,
				(sizeof(gyroSensorCharacteristicValues) / sizeof(uint8_t)),
				(uint8_t *) &gyroSensorCharacteristicValues[i], 0, 0,
				&gyroSensorService,
				&gyroCharacteristicProperties[i].characteristicAttribute) == BLESTATUS_FAILED) {
			ATT_SERVER_ReleaseDatabaseAccess();
			return BLESTATUS_FAILED;
		}

		ATT_SERVER_ReleaseDatabaseAccess();
	}

	return BLESTATUS_SUCCESS;
}

static BleStatus vxaBleRegisterLightSensor(void) {
	ATT_SERVER_SecureDatabaseAccess();

	if (ATT_SERVER_RegisterServiceAttribute(
	ATTPDU_SIZEOF_128_BIT_UUID, (uint8_t *) lightSensorServiceUUID,
			lightSensorServiceCallback,
			&(lightSensorService)) == BLESTATUS_FAILED) {
		ATT_SERVER_ReleaseDatabaseAccess();
		return BLESTATUS_FAILED;
	}

	lightSensorCharacteristicProperties.uuidType.size = ATT_UUID_SIZE_128;
	lightSensorCharacteristicProperties.uuidType.value.uuid128 =
			(uint8_t *) &lightSensorCharacteristicProperties.characteristicUUID;

	lightSensorCharacteristicValue[0] = 123;

	if (ATT_SERVER_AddCharacteristic(
	ATTPROPERTY_READ | ATTPROPERTY_NOTIFY,
			(Att16BitCharacteristicAttribute*) &lightSensorCharacteristicProperties.characteristic,
			&lightSensorCharacteristicProperties.uuidType,
			ATT_PERMISSIONS_ALLACCESS,
			(sizeof(lightSensorCharacteristicValue) / sizeof(uint8_t)),
			(uint8_t *) &lightSensorCharacteristicValue[0], 0, 0,
			&lightSensorService,
			&lightSensorCharacteristicProperties.characteristicAttribute) == BLESTATUS_FAILED) {
		ATT_SERVER_ReleaseDatabaseAccess();
		return BLESTATUS_FAILED;
	}

	ATT_SERVER_ReleaseDatabaseAccess();

	return BLESTATUS_SUCCESS;
}

static BleStatus vxaBleRegisterNoiseSensor(void) {
	ATT_SERVER_SecureDatabaseAccess();

	if (ATT_SERVER_RegisterServiceAttribute(
	ATTPDU_SIZEOF_128_BIT_UUID, (uint8_t *) noiseSensorServiceUUID,
			sensorServicesCallback, &(noiseSensorService)) == BLESTATUS_FAILED) {
		ATT_SERVER_ReleaseDatabaseAccess();
		return BLESTATUS_FAILED;
	}

	noiseSensorCharacteristicProperties.uuidType.size = ATT_UUID_SIZE_128;
	noiseSensorCharacteristicProperties.uuidType.value.uuid128 =
			(uint8_t *) &noiseSensorCharacteristicProperties.characteristicUUID;

	noiseSensorCharacteristicValue[0] = 10;

	if (ATT_SERVER_AddCharacteristic(
	ATTPROPERTY_READ | ATTPROPERTY_NOTIFY,
			(Att16BitCharacteristicAttribute*) &noiseSensorCharacteristicProperties.characteristic,
			&noiseSensorCharacteristicProperties.uuidType,
			ATT_PERMISSIONS_ALLACCESS,
			(sizeof(noiseSensorCharacteristicValue) / sizeof(uint8_t)),
			(uint8_t *) &noiseSensorCharacteristicValue[0], 0, 0,
			&noiseSensorService,
			&noiseSensorCharacteristicProperties.characteristicAttribute) == BLESTATUS_FAILED) {
		ATT_SERVER_ReleaseDatabaseAccess();
		return BLESTATUS_FAILED;
	}

	ATT_SERVER_ReleaseDatabaseAccess();

	return BLESTATUS_SUCCESS;
}

static BleStatus vxaBleRegisterMagnetometerSensor(void) {
	ATT_SERVER_SecureDatabaseAccess();

	if (ATT_SERVER_RegisterServiceAttribute(
	ATTPDU_SIZEOF_128_BIT_UUID, (uint8_t *) magnetometerSensorServiceUUID,
			sensorServicesCallback,
			&(magnetometerSensorService)) == BLESTATUS_FAILED) {
		ATT_SERVER_ReleaseDatabaseAccess();
		return BLESTATUS_FAILED;
	}

	ATT_SERVER_ReleaseDatabaseAccess();

	/* Add the magnetometer values for the 3 axis to the attribute database */
	for (uint8_t i = 0; i < 4; i++) {
		ATT_SERVER_SecureDatabaseAccess();

		magnetometerSensorCharacteristicProperties[i].uuidType.size =
				ATT_UUID_SIZE_128;
		magnetometerSensorCharacteristicProperties[i].uuidType.value.uuid128 =
				(uint8_t *) &magnetometerSensorCharacteristicProperties[i].characteristicUUID;

		magnetometerSensorCharacteristicValues[i][0] = 10 + i;

		if (ATT_SERVER_AddCharacteristic(
		ATTPROPERTY_READ | ATTPROPERTY_NOTIFY,
				(Att16BitCharacteristicAttribute*) &magnetometerSensorCharacteristicProperties[i].characteristic,
				&magnetometerSensorCharacteristicProperties[i].uuidType,
				ATT_PERMISSIONS_ALLACCESS, 2,
				(uint8_t *) &magnetometerSensorCharacteristicValues[i][0], 0, 0,
				&gyroSensorService,
				&magnetometerSensorCharacteristicProperties[i].characteristicAttribute) == BLESTATUS_FAILED) {
			ATT_SERVER_ReleaseDatabaseAccess();
			return BLESTATUS_FAILED;
		}

		ATT_SERVER_ReleaseDatabaseAccess();
	}
	return BLESTATUS_SUCCESS;
}

static BleStatus vxaBleRegisterEnvironmnetSensor(void) {
	ATT_SERVER_SecureDatabaseAccess();

	if (ATT_SERVER_RegisterServiceAttribute(
	ATTPDU_SIZEOF_128_BIT_UUID, (uint8_t *) environmentSensorServiceUUID,
			sensorServicesCallback,
			&(environmentSensorService)) == BLESTATUS_FAILED) {
		ATT_SERVER_ReleaseDatabaseAccess();
		return BLESTATUS_FAILED;
	}

	/*------- register presseure sensor characteristic ---------------------------------------------*/

	environmentSensorPressureCharacteristicProperties.uuidType.size =
			ATT_UUID_SIZE_128;
	environmentSensorPressureCharacteristicProperties.uuidType.value.uuid128 =
			(uint8_t *) &environmentSensorPressureCharacteristicProperties.characteristicUUID;

	environmentSensorPressureCharacteristicValue[0] = 10;

	if (ATT_SERVER_AddCharacteristic(
	ATTPROPERTY_READ | ATTPROPERTY_NOTIFY,
			(Att16BitCharacteristicAttribute*) &environmentSensorPressureCharacteristicProperties.characteristic,
			&environmentSensorPressureCharacteristicProperties.uuidType,
			ATT_PERMISSIONS_ALLACCESS, 2,
			(uint8_t *) &environmentSensorPressureCharacteristicValue[0], 0, 0,
			&environmentSensorService,
			&environmentSensorPressureCharacteristicProperties.characteristicAttribute) == BLESTATUS_FAILED) {
		ATT_SERVER_ReleaseDatabaseAccess();
		return BLESTATUS_FAILED;
	}

	/*------- register temperature sensor characteristic ---------------------------------------------*/

	environmentSensorTemperatureCharacteristicProperties.uuidType.size =
			ATT_UUID_SIZE_128;
	environmentSensorTemperatureCharacteristicProperties.uuidType.value.uuid128 =
			(uint8_t *) &environmentSensorTemperatureCharacteristicProperties.characteristicUUID;

	environmentSensorTemperatureCharacteristicValue[0] = 10;

	if (ATT_SERVER_AddCharacteristic(
	ATTPROPERTY_READ | ATTPROPERTY_NOTIFY,
			(Att16BitCharacteristicAttribute*) &environmentSensorTemperatureCharacteristicProperties.characteristic,
			&environmentSensorTemperatureCharacteristicProperties.uuidType,
			ATT_PERMISSIONS_ALLACCESS, 2,
			(uint8_t *) &environmentSensorTemperatureCharacteristicValue[0], 0,
			0, &environmentSensorService,
			&environmentSensorTemperatureCharacteristicProperties.characteristicAttribute) == BLESTATUS_FAILED) {
		ATT_SERVER_ReleaseDatabaseAccess();
		return BLESTATUS_FAILED;
	}

	/*------- register humidity sensor characteristic ---------------------------------------------*/

	environmentSensorHumidityCharacteristicProperties.uuidType.size =
			ATT_UUID_SIZE_128;
	environmentSensorHumidityCharacteristicProperties.uuidType.value.uuid128 =
			(uint8_t *) &environmentSensorHumidityCharacteristicProperties.characteristicUUID;

	environmentSensorHumidityCharacteristicValue[0] = 10;

	if (ATT_SERVER_AddCharacteristic(
	ATTPROPERTY_READ | ATTPROPERTY_NOTIFY,
			(Att16BitCharacteristicAttribute*) &environmentSensorHumidityCharacteristicProperties.characteristic,
			&environmentSensorHumidityCharacteristicProperties.uuidType,
			ATT_PERMISSIONS_ALLACCESS, 2,
			(uint8_t *) &environmentSensorHumidityCharacteristicValue[0], 0, 0,
			&environmentSensorService,
			&environmentSensorHumidityCharacteristicProperties.characteristicAttribute) == BLESTATUS_FAILED) {
		ATT_SERVER_ReleaseDatabaseAccess();
		return BLESTATUS_FAILED;
	}

	ATT_SERVER_ReleaseDatabaseAccess();

	return BLESTATUS_SUCCESS;
}

static BleStatus vxaBleRegisterXdkControl(void) {
	ATT_SERVER_SecureDatabaseAccess();

	if (ATT_SERVER_RegisterServiceAttribute(
	ATTPDU_SIZEOF_128_BIT_UUID, (uint8_t *) controlXdkServiceUUID,
			controlXdkServiceCallback, &(controlXdkService)) == BLESTATUS_FAILED) {
		ATT_SERVER_ReleaseDatabaseAccess();
		return BLESTATUS_FAILED;
	}

	//-------------------------Start sensor sampling Characteristic-------------------------------------
	startSensorSamplingAndNotificationsCharacteristicProperties.uuidType.size =
			ATT_UUID_SIZE_128;
	startSensorSamplingAndNotificationsCharacteristicProperties.uuidType.value.uuid128 =
			(uint8_t *) &startSensorSamplingAndNotificationsCharacteristicProperties.characteristicUUID;

	startSensorSamplingCharacteristicValue[0] = 0;

	if (ATT_SERVER_AddCharacteristic(
	ATTPROPERTY_WRITE,
			(Att16BitCharacteristicAttribute*) &startSensorSamplingAndNotificationsCharacteristicProperties.characteristic,
			&startSensorSamplingAndNotificationsCharacteristicProperties.uuidType,
			ATT_PERMISSIONS_ALLACCESS,
			(sizeof(startSensorSamplingCharacteristicValue) / sizeof(uint8_t)),
			(uint8_t *) &startSensorSamplingCharacteristicValue[0], 0, 0,
			&controlXdkService,
			&startSensorSamplingAndNotificationsCharacteristicProperties.characteristicAttribute) == BLESTATUS_FAILED) {
		ATT_SERVER_ReleaseDatabaseAccess();
		return BLESTATUS_FAILED;
	}

	//-------------------------Sampling Rate Characteristic-------------------------------------
	setSamplingRateCharacteristicProperties.uuidType.size = ATT_UUID_SIZE_128;
	setSamplingRateCharacteristicProperties.uuidType.value.uuid128 =
			(uint8_t *) &setSamplingRateCharacteristicProperties.characteristicUUID;

	setSensorSamplingTimerCharacteristicValue[0] = 0;

	if (ATT_SERVER_AddCharacteristic(
	ATTPROPERTY_WRITE,
			(Att16BitCharacteristicAttribute*) &setSamplingRateCharacteristicProperties.characteristic,
			&setSamplingRateCharacteristicProperties.uuidType,
			ATT_PERMISSIONS_ALLACCESS,
			(sizeof(setSensorSamplingTimerCharacteristicValue) / sizeof(uint8_t)),
			(uint8_t *) &setSensorSamplingTimerCharacteristicValue[0], 0, 0,
			&controlXdkService,
			&setSamplingRateCharacteristicProperties.characteristicAttribute) == BLESTATUS_FAILED) {
		ATT_SERVER_ReleaseDatabaseAccess();
		return BLESTATUS_FAILED;
	}

	//-------------------------Reset XDK Characteristic-------------------------------------
	resetXdkCharacteristicProperties.uuidType.size = ATT_UUID_SIZE_128;
	resetXdkCharacteristicProperties.uuidType.value.uuid128 =
			(uint8_t *) &resetXdkCharacteristicProperties.characteristicUUID;

	resetXdkCharacteristicValue[0] = 0;

	if (ATT_SERVER_AddCharacteristic(
	ATTPROPERTY_WRITE,
			(Att16BitCharacteristicAttribute*) &resetXdkCharacteristicProperties.characteristic,
			&resetXdkCharacteristicProperties.uuidType,
			ATT_PERMISSIONS_ALLACCESS,
			(sizeof(resetXdkCharacteristicValue) / sizeof(uint8_t)),
			(uint8_t *) &resetXdkCharacteristicValue[0], 0, 0,
			&controlXdkService,
			&resetXdkCharacteristicProperties.characteristicAttribute) == BLESTATUS_FAILED) {
		ATT_SERVER_ReleaseDatabaseAccess();
		return BLESTATUS_FAILED;
	}

	//-------------------------XDK Firmware Characteristic-------------------------------------

	getXdkFirmwareVersionCharacteristicProperties.uuidType.size =
			ATT_UUID_SIZE_128;
	getXdkFirmwareVersionCharacteristicProperties.uuidType.value.uuid128 =
			(uint8_t *) &getXdkFirmwareVersionCharacteristicProperties.characteristicUUID;

	if (ATT_SERVER_AddCharacteristic(
	ATTPROPERTY_WRITE,
			(Att16BitCharacteristicAttribute*) &getXdkFirmwareVersionCharacteristicProperties.characteristic,
			&getXdkFirmwareVersionCharacteristicProperties.uuidType,
			ATT_PERMISSIONS_ALLACCESS,
			(sizeof(firmwareVersionCharacteristicValue) / sizeof(uint8_t)),
			(uint8_t *) &firmwareVersionCharacteristicValue[0], 0, 0,
			&controlXdkService,
			&getXdkFirmwareVersionCharacteristicProperties.characteristicAttribute) == BLESTATUS_FAILED) {
		ATT_SERVER_ReleaseDatabaseAccess();
		return BLESTATUS_FAILED;
	}

	ATT_SERVER_ReleaseDatabaseAccess();

	return BLESTATUS_SUCCESS;
}

static BleStatus vxaBleRegisterHighDataRateService(void) {
	ATT_SERVER_SecureDatabaseAccess();

	if (ATT_SERVER_RegisterServiceAttribute(
	ATTPDU_SIZEOF_128_BIT_UUID, (uint8_t *) highDataRateServiceUUID,
			highDataRateServiceCallback,
			&(highDataRateService)) == BLESTATUS_FAILED) {
		ATT_SERVER_ReleaseDatabaseAccess();
		return BLESTATUS_FAILED;
	}

	ATT_SERVER_ReleaseDatabaseAccess();

	for (uint8_t i = 0; i < 2; i++) {
		ATT_SERVER_SecureDatabaseAccess();

		highDataRateCharacteristicProperties[i].uuidType.size =
				ATT_UUID_SIZE_128;
		highDataRateCharacteristicProperties[i].uuidType.value.uuid128 =
				(uint8_t *) &highDataRateCharacteristicProperties[i].characteristicUUID;

		highDataRateCharacteristicValue[i][0] = 10 + i;

		if (ATT_SERVER_AddCharacteristic(
		ATTPROPERTY_READ | ATTPROPERTY_NOTIFY,
				(Att16BitCharacteristicAttribute*) &highDataRateCharacteristicProperties[i].characteristic,
				&highDataRateCharacteristicProperties[i].uuidType,
				ATT_PERMISSIONS_ALLACCESS, 20,
				(uint8_t *) &highDataRateCharacteristicValue[i][0], 0, 0,
				&highDataRateService,
				&highDataRateCharacteristicProperties[i].characteristicAttribute) == BLESTATUS_FAILED) {
			ATT_SERVER_ReleaseDatabaseAccess();
			return BLESTATUS_FAILED;
		}

		ATT_SERVER_ReleaseDatabaseAccess();
	}
	return BLESTATUS_SUCCESS;
}

static void acceleratorSensorServiceCallback(
		AttServerCallbackParms *serverCallbackParms) {
	switch (serverCallbackParms->event) {
	case ATTEVT_SERVER_READ_REQ: {
		printf("ATTEVT_SERVER_READ_REQ event found\n");
		if (serverCallbackParms->parms.readReq.attribute
				== &accelerometerCharacteristicProperties[0].characteristicAttribute) {
			//peer = serverCallbackParms->peer;
			//printf("Peer: %d\n", BLE_returnConnectionHandle());
		}
	}
		break;
	case ATTEVT_SERVER_HVI_SENT: {
		printf("ATTEVT_SERVER_HVI_SENT event found\n");
		if (serverCallbackParms->parms.readReq.attribute
				== &accelerometerCharacteristicProperties[0].characteristicAttribute) {
			printf("Accelerometer X-Axis");
		}
		if (serverCallbackParms->parms.readReq.attribute
				== &accelerometerCharacteristicProperties[1].characteristicAttribute) {
			printf("Accelerometer Y-Axis");
		}
		if (serverCallbackParms->parms.readReq.attribute
				== &accelerometerCharacteristicProperties[2].characteristicAttribute) {
			printf("Accelerometer Z-Axis");
		}
	}
		break;
	default:
		break;
	}
}

static void gyroSensorServiceCallback(
		AttServerCallbackParms* serverCallbackParms) {
	switch (serverCallbackParms->event) {
	case ATTEVT_SERVER_READ_REQ: {
		printf("ATTEVT_SERVER_READ_REQ event found\n");
		if (serverCallbackParms->parms.readReq.attribute
				== &gyroCharacteristicProperties[0].characteristicAttribute) {
			//peer = serverCallbackParms->peer;
			//printf("Peer: %d\n", BLE_returnConnectionHandle());
		}
	}
		break;
	case ATTEVT_SERVER_HVI_SENT: {
		printf("ATTEVT_SERVER_HVI_SENT event found\n");

		if (serverCallbackParms->parms.readReq.attribute
				== &gyroCharacteristicProperties[0].characteristicAttribute) {
			printf("Gyro X-Axis\n");
		}
		if (serverCallbackParms->parms.readReq.attribute
				== &gyroCharacteristicProperties[1].characteristicAttribute) {
			printf("Gyro Y-Axis\n");
		}
		if (serverCallbackParms->parms.readReq.attribute
				== &gyroCharacteristicProperties[2].characteristicAttribute) {
			printf("Gyro Y-Axis\n");
		}

	}
		break;
	default:
		break;
	}
}

static void lightSensorServiceCallback(
		AttServerCallbackParms* serverCallbackParms) {
	switch (serverCallbackParms->event) {
	case ATTEVT_SERVER_READ_REQ: {
		printf("ATTEVT_SERVER_READ_REQ event found\n");
		if (serverCallbackParms->parms.readReq.attribute
				== &lightSensorCharacteristicProperties.characteristicAttribute) {
			//peer = serverCallbackParms->peer;
			//printf("Peer: %d\n", BLE_returnConnectionHandle());
		}
	}
		break;
	case ATTEVT_SERVER_HVI_SENT: {
		printf("ATTEVT_SERVER_HVI_SENT event found for the\n");
		if (serverCallbackParms->parms.readReq.attribute
				== &lightSensorCharacteristicProperties.characteristicAttribute) {
			printf("Light sensor\n");
		}

	}
		break;
	default:
		break;
	}
}

static void sensorServicesCallback(AttServerCallbackParms* serverCallbackParms) {
	switch (serverCallbackParms->event) {
	case ATTEVT_SERVER_READ_REQ: {
		printf("ATTEVT_SERVER_READ_REQ event found\n");
	}
		break;
	case ATTEVT_SERVER_HVI_SENT: {
		printf("ATTEVT_SERVER_HVI_SENT event found\n");
	}
		break;
	default:
		break;
	}
}

static void highDataRateServiceCallback(
		AttServerCallbackParms* serverCallbackParms) {
	switch (serverCallbackParms->event) {
	case ATTEVT_SERVER_READ_REQ: {

	}
		break;
	case ATTEVT_SERVER_WRITE_REQ: {

	}
		break;
	case ATTEVT_SERVER_WRITE_COMPLETE: {

	}
		break;
	default:
		break;
	}
}

static void controlXdkServiceCallback(
		AttServerCallbackParms* serverCallbackParms) {
	switch (serverCallbackParms->event) {
	case ATTEVT_SERVER_READ_REQ: {
		printf("Control XDK Callback: ATTEVT_SERVER_READ_REQ event found\n");
	}
		break;
	case ATTEVT_SERVER_WRITE_REQ: {
		printf("Control XDK Callback:ATTEVT_SERVER_WRITE_REQ event found\n");

	}
		break;
	case ATTEVT_SERVER_WRITE_COMPLETE: {
		printf(
				"Control XDK Callback:ATTEVT_SERVER_WRITE_COMPLETE event found\n");
		if (serverCallbackParms->parms.writeComplete.attribute
				== &startSensorSamplingAndNotificationsCharacteristicProperties.characteristicAttribute) {
			uint8_t value;
			const uint8_t* pValue = &value;
			printf("Should we start the timer here?\n");

			pValue = serverCallbackParms->parms.writeComplete.value;
			printf("Received Value: %d\n", *pValue);

			if (toggleTimerOnOff == 0) {

				if (xTimerStart(VXA_getHighDataRateTimerHandle(),
						0xffff)!=pdTRUE) {
					assert(0);
				}
			}
			if (toggleTimerOnOff == 1) {

				if (xTimerStop(VXA_getHighDataRateTimerHandle(),
						0xffff)==pdFAIL) {
					assert(0);
				}
			}
		}
		if (serverCallbackParms->parms.writeComplete.attribute
				== &setSamplingRateCharacteristicProperties.characteristicAttribute) {
			uint32_t receivedSamplingRate = 0;

			uint8_t receiveBuffer[4];

			memset(receiveBuffer, 0, (sizeof(receiveBuffer) / sizeof(uint8_t)));

			receiveBuffer[0] =
					*(serverCallbackParms->parms.writeComplete.value);
			receiveBuffer[1] = *(serverCallbackParms->parms.writeComplete.value
					+ 1);
			receiveBuffer[2] = *(serverCallbackParms->parms.writeComplete.value
					+ 2);
			receiveBuffer[3] = *(serverCallbackParms->parms.writeComplete.value
					+ 3);

			printf("Sampling rate receiveBuffer[0]: %d\n", receiveBuffer[0]);
			printf("Sampling rate receiveBuffer[1]: %d\n", receiveBuffer[1]);
			printf("Sampling rate receiveBuffer[2]: %d\n", receiveBuffer[2]);
			printf("Sampling rate receiveBuffer[3]: %d\n", receiveBuffer[3]);

			receivedSamplingRate = receiveBuffer[3];
			receivedSamplingRate = (receivedSamplingRate << 8)
					+ receiveBuffer[2];
			receivedSamplingRate = (receivedSamplingRate << 8)
					+ receiveBuffer[1];
			receivedSamplingRate = (receivedSamplingRate << 8)
					+ receiveBuffer[0];

			printf("New sensor sampling rate received\n");
			printf("New sampling rate: %d\n", (int) receivedSamplingRate);

			if ((receivedSamplingRate < 20) || (receivedSamplingRate > 5000)) {
				receivedSamplingRate = 100;
			}

			setHighDataRateSensorSamplingRate(receivedSamplingRate);
		}
		if (serverCallbackParms->parms.writeComplete.attribute
				== &resetXdkCharacteristicProperties.characteristicAttribute) {
			printf("Reset request received from App! \n");
			NVIC_SystemReset();
		}
	}
		break;
	default:
		break;
	}
}

static void bluetoothConnectedNotificationCallback(
		BLE_connectionDetails_t connectionDetails) {

	switch (connectionDetails.connectionStatus) {
	case BLE_CONNECTED_TO_DEVICE:
		printf("Device connected  : \r\n");
		//        peer = BLE_returnConnectionHandle();
		//        printf("Peer: %d\n", peer);
		peer = 1;
		/** TODO: Use LED driver instead! */
		PTD_pinOutSet((GPIO_Port_TypeDef) PTD_PORT_LED_ORANGE,
				PTD_PIN_LED_ORANGE);
		break;
	case BLE_DISCONNECTED_FROM_DEVICE:
		//        printf("Device Disconnected   : \r\n");
		//        peer = BLE_returnConnectionHandle();
		printf("Peer: %d\n", peer);
		/** TODO: Use LED driver instead! */
		PTD_pinOutClear((GPIO_Port_TypeDef) PTD_PORT_LED_ORANGE,
				PTD_PIN_LED_ORANGE);
		uint32_t Ticks = 0xffff;

		if (Ticks != UINT32_MAX) /* Validated for portMAX_DELAY to assist the task to wait Infinitely (without timing out) */
		{
			Ticks /= portTICK_RATE_MS;
		}
		if (xTimerStop(VXA_getHighDataRateTimerHandle(), Ticks) != pdTRUE) {
			assert(0);
		}
		break;
	default:
		assert(0);
		break;
	}
}

static void printSendNotificationError(BleStatus status) {
	if (status == BLESTATUS_SUCCESS) {
		printf("SendIndication status: BLESTATUS_SUCCESS\n");

	} else if (status == BLESTATUS_FAILED) {
		printf("SendIndication status: BLESTATUS_FAILED\n");
	} else if (status == BLESTATUS_INVALID_PARMS) {
		printf("SendIndication status: BLESTATUS_INVALID_PARMS\n");

	} else if (status == BLESTATUS_PENDING) {
		printf("SendIndication status: BLESTATUS_PENDING\n");
	} else if (status == BLESTATUS_BUSY) {
		printf("SendIndication status: BLESTATUS_BUSY\n");
	} else {
		printf("SendIndication status: other Error!\n");
	}
}

/* global functions ********************************************************* */

extern void VXA_bluetoothLE_writeAndNotifyAccelerometerAxisValue(int16_t value,
		Sensor_Axis_t axis) {

	BleStatus status;

	acceleratorSensorCharacteristicValues[axis][0] = (uint8_t) value;
	acceleratorSensorCharacteristicValues[axis][1] = (uint8_t) (value >> 8);

	ATT_SERVER_SecureDatabaseAccess();

	status =
			ATT_SERVER_WriteAttributeValue(
					&accelerometerCharacteristicProperties[axis].characteristicAttribute,
					(const uint8_t *) &acceleratorSensorCharacteristicValues[axis][0],
					(sizeof(acceleratorSensorCharacteristicValues[axis])
							/ sizeof(uint8_t)));

#if ENABLE_ACCELEROMETER_NOTIFICATIONS

	if (status == BLESTATUS_SUCCESS) /* send notification */
	{
		status = ATT_SERVER_SendNotification(
				&accelerometerCharacteristicProperties[axis].characteristicAttribute,
				peer);
		printf("Accelerometer sensor notification, axis: %i \n", axis);
		printSendNotificationError(status);
	}
	else
	{
		assert(status != BLESTATUS_FAILED);

	}

#endif

	ATT_SERVER_ReleaseDatabaseAccess();

#if !ENABLE_ACCELEROMETER_NOTIFICATIONS
	(void) status;
#endif

}

extern void VXA_bluetoothLE_writeAndNotifyGyroAxisValue(int16_t value,
		Sensor_Axis_t axis) {
	BleStatus status;

	gyroSensorCharacteristicValues[axis][0] = (uint8_t) value;
	gyroSensorCharacteristicValues[axis][1] = (uint8_t) (value >> 8);

	ATT_SERVER_SecureDatabaseAccess();

	status = ATT_SERVER_WriteAttributeValue(
			&gyroCharacteristicProperties[axis].characteristicAttribute,
			(const uint8_t *) &gyroSensorCharacteristicValues[axis][0],
			(sizeof(gyroSensorCharacteristicValues) / sizeof(uint8_t)));

#if ENABLE_GYRO_NOTIFICATIONS

	if (status == BLESTATUS_SUCCESS) /* send notification */
	{
		status = ATT_SERVER_SendNotification(
				&gyroCharacteristicProperties[axis].characteristicAttribute,
				peer);
		printf("Gyro sensor notification, axis: %i \n", 0);
		printSendNotificationError(status);
	}
	else
	{
		assert(status != BLESTATUS_FAILED);

	}

#endif

	ATT_SERVER_ReleaseDatabaseAccess();

#if !ENABLE_ACCELEROMETER_NOTIFICATIONS
	(void) status;
#endif
}

extern void VXA_bluetoothLE_writeAndNotifyLightSensorValue(uint32_t value) {
	BleStatus status = BLESTATUS_FAILED;

	lightSensorCharacteristicValue[0] = (uint8_t) value;
	lightSensorCharacteristicValue[1] = (uint8_t) (value >> 8);
	lightSensorCharacteristicValue[2] = (uint8_t) (value >> 16);
	lightSensorCharacteristicValue[3] = (uint8_t) (value >> 24);

	lightSensorCharacteristicValue[0]++;
	printf("Light sensor write value: %i\n", lightSensorCharacteristicValue[0]);

	ATT_SERVER_SecureDatabaseAccess();

	status = ATT_SERVER_WriteAttributeValue(
			&lightSensorCharacteristicProperties.characteristicAttribute,
			(const uint8_t *) &lightSensorCharacteristicValue[0],
			sizeof(lightSensorCharacteristicValue) / sizeof(uint8_t));

#if ENABLE_LIGHT_SENSOR_NOTIFICATIONS

	if (status == BLESTATUS_SUCCESS) /* send notification */
	{
		status = ATT_SERVER_SendNotification(
				&lightSensorCharacteristicProperties.characteristicAttribute,
				peer);
		printf("Light sensor notification \n");
		printSendNotificationError(status);
	}
	else
	{
		assert(status != BLESTATUS_FAILED);

	}
#endif

	ATT_SERVER_ReleaseDatabaseAccess();

#if !ENABLE_ACCELEROMETER_NOTIFICATIONS
	(void) status;
#endif
}

extern void VXA_bluetoothLE_writeAndNotifyNoiseSensorValue(uint8_t value) {
	BleStatus status;

	noiseSensorCharacteristicValue[0] = (uint8_t) value;

	ATT_SERVER_SecureDatabaseAccess();

	status = ATT_SERVER_WriteAttributeValue(
			&noiseSensorCharacteristicProperties.characteristicAttribute,
			(void*) (intptr_t) noiseSensorCharacteristicValue[0],
			sizeof(noiseSensorCharacteristicValue) / sizeof(uint8_t));

#if	ENABLE_NOISE_SENSOR_NOTIFICATIONS

	if (status == BLESTATUS_SUCCESS) /* send notification */
	{
		status = ATT_SERVER_SendNotification(
				&noiseSensorCharacteristicProperties.characteristicAttribute,
				peer);
		printf("Noise sensor notification \n");
		printSendNotificationError(status);
	}
	else
	{
		assert(status != BLESTATUS_FAILED);

	}

#endif

	ATT_SERVER_ReleaseDatabaseAccess();

#if !ENABLE_ACCELEROMETER_NOTIFICATIONS
	(void) status;
#endif
}

extern void VXA_bluetoothLE_writeAndNotifyMagnetometerAxisValue(int16_t value,
		Sensor_Axis_t axis) {
	BleStatus status;

	magnetometerSensorCharacteristicValues[axis][0] = (uint8_t) (value);
	magnetometerSensorCharacteristicValues[axis][1] = (uint8_t) (value >> 8);

	ATT_SERVER_SecureDatabaseAccess();

	status =
			ATT_SERVER_WriteAttributeValue(
					&magnetometerSensorCharacteristicProperties[axis].characteristicAttribute,
					(const uint8_t *) &magnetometerSensorCharacteristicValues[axis][0],
					(sizeof(magnetometerSensorCharacteristicValues[axis])
							/ sizeof(uint8_t)));

#if ENABLE_MAGNETOMETER_NOTIFICATIONS

	if (status == BLESTATUS_SUCCESS) /* send notification */
	{
		status = ATT_SERVER_SendNotification(
				&magnetometerSensorCharacteristicProperties[axis].characteristicAttribute,
				peer);
		printf("Magnetometer sensor notification \n");
		printSendNotificationError(status);
	}
	else
	{
		assert(status != BLESTATUS_FAILED);

	}

#endif

	ATT_SERVER_ReleaseDatabaseAccess();

#if !ENABLE_ACCELEROMETER_NOTIFICATIONS
	(void) status;
#endif
}

extern void VXA_bluetoothLE_writeAndNotifyMagnetometerResistanceValue(
		int16_t value) {
	BleStatus status;

	magnetometerSensorCharacteristicValues[4][0] = (uint8_t) value;
	magnetometerSensorCharacteristicValues[4][1] = (uint8_t) (value >> 8);

	ATT_SERVER_SecureDatabaseAccess();

	status =
			ATT_SERVER_WriteAttributeValue(
					&magnetometerSensorCharacteristicProperties[4].characteristicAttribute,
					(const uint8_t *) &magnetometerSensorCharacteristicValues[4][0],
					(sizeof(magnetometerSensorCharacteristicValues[4])
							/ sizeof(uint8_t)));

#if ENABLE_MAGNETOMETER_NOTIFICATIONS

	if (status == BLESTATUS_SUCCESS) /* send notification */
	{
		status = ATT_SERVER_SendNotification(
				&magnetometerSensorCharacteristicProperties[4].characteristicAttribute,
				peer);
		printf("Gyro sensor notification \n");
		printSendNotificationError(status);
	}
	else
	{
		assert(status != BLESTATUS_FAILED);

	}

#endif

	ATT_SERVER_ReleaseDatabaseAccess();

#if !ENABLE_ACCELEROMETER_NOTIFICATIONS
	(void) status;
#endif
}

extern void VXA_bluetoothLE_writeAndNotifyPressureSensorValue(uint32_t value) {
	BleStatus status;

	environmentSensorPressureCharacteristicValue[0] = (uint8_t) value;
	environmentSensorPressureCharacteristicValue[1] = (uint8_t) (value >> 8);
	environmentSensorPressureCharacteristicValue[2] = (uint8_t) (value >> 16);
	environmentSensorPressureCharacteristicValue[3] = (uint8_t) (value >> 24);

	ATT_SERVER_SecureDatabaseAccess();

	status =
			ATT_SERVER_WriteAttributeValue(
					&environmentSensorPressureCharacteristicProperties.characteristicAttribute,
					(const uint8_t *) &environmentSensorPressureCharacteristicValue[0],
					4);

#if ENABLE_ENVIRONMENT_SENSOR_NOTIFICATIONS

	if (status == BLESTATUS_SUCCESS) /* send notification */
	{
		status = ATT_SERVER_SendNotification(
				&environmentSensorPressureCharacteristicProperties.characteristicAttribute,
				peer);
		printf("Light sensor notification \n");
		printSendNotificationError(status);
	}
	else
	{
		assert(status != BLESTATUS_FAILED);

	}

#endif

	ATT_SERVER_ReleaseDatabaseAccess();

#if !ENABLE_ACCELEROMETER_NOTIFICATIONS
	(void) status;
#endif
}

extern void VXA_bluetoothLE_writeAndNotifyTemperatureSensorValue(int32_t value) {
	BleStatus status;

	environmentSensorTemperatureCharacteristicValue[0] = (uint8_t) value;
	environmentSensorTemperatureCharacteristicValue[1] = (uint8_t) (value >> 8);
	environmentSensorTemperatureCharacteristicValue[2] =
			(uint8_t) (value >> 16);
	environmentSensorTemperatureCharacteristicValue[3] =
			(uint8_t) (value >> 24);

	ATT_SERVER_SecureDatabaseAccess();

	status =
			ATT_SERVER_WriteAttributeValue(
					&environmentSensorTemperatureCharacteristicProperties.characteristicAttribute,
					(const uint8_t *) &environmentSensorTemperatureCharacteristicValue[0],
					4);

#if ENABLE_ENVIRONMENT_SENSOR_NOTIFICATIONS

	if (status == BLESTATUS_SUCCESS) /* send notification */
	{
		status = ATT_SERVER_SendNotification(
				&environmentSensorTemperatureCharacteristicProperties.characteristicAttribute,
				peer);
		printf("Light sensor notification \n");
		printSendNotificationError(status);
	}
	else
	{
		assert(status != BLESTATUS_FAILED);

	}

#endif

	ATT_SERVER_ReleaseDatabaseAccess();

#if !ENABLE_ACCELEROMETER_NOTIFICATIONS
	(void) status;
#endif
}

extern void VXA_bluetoothLE_writeAndNotifyHumiditySensorValue(uint32_t value) {
	BleStatus status;

	environmentSensorHumidityCharacteristicValue[0] = (uint8_t) value;
	environmentSensorHumidityCharacteristicValue[1] = (uint8_t) (value >> 8);
	environmentSensorHumidityCharacteristicValue[2] = (uint8_t) (value >> 16);
	environmentSensorHumidityCharacteristicValue[3] = (uint8_t) (value >> 24);

	ATT_SERVER_SecureDatabaseAccess();

	status =
			ATT_SERVER_WriteAttributeValue(
					&environmentSensorHumidityCharacteristicProperties.characteristicAttribute,
					(const uint8_t *) &environmentSensorHumidityCharacteristicValue[0],
					4);

#if !ENABLE_ENVIRONMENT_SENSOR_NOTIFICATIONS
	(void) status;
#endif

#if ENABLE_ENVIRONMENT_SENSOR_NOTIFICATIONS

	if (status == BLESTATUS_SUCCESS) /* send notification */
	{
		status = ATT_SERVER_SendNotification(
				&environmentSensorHumidityCharacteristicProperties.characteristicAttribute,
				peer);
		printf("Light sensor notification \n");
		printSendNotificationError(status);
	}
	else
	{
		assert(status != BLESTATUS_FAILED);

	}

#endif

	ATT_SERVER_ReleaseDatabaseAccess();

#if !ENABLE_ACCELEROMETER_NOTIFICATIONS
	(void) status;
#endif
}

extern void VXA_bluetoothLE_writeAndNotifyHighPriorityData(uint8_t* buffer) {
	BleStatus status;

	memcpy(highDataRateCharacteristicValue[HIGH_PRIORITY_DATA], buffer, 20);

	ATT_SERVER_SecureDatabaseAccess();

	status =
			ATT_SERVER_WriteAttributeValue(
					&highDataRateCharacteristicProperties[HIGH_PRIORITY_DATA].characteristicAttribute,
					(const uint8_t *) &highDataRateCharacteristicValue[HIGH_PRIORITY_DATA][0],
					20);

#if ENABLE_HIGH_PRIO_DATA_SERVICE

	if (status == BLESTATUS_SUCCESS) /* send notification */
	{
		status =
				ATT_SERVER_SendNotification(
						&highDataRateCharacteristicProperties[HIGH_PRIORITY_DATA].characteristicAttribute,
						peer);
		//printf("high priority sensor data notification \n");
		printSendNotificationError(status);
	} else {
		assert(status != BLESTATUS_FAILED);

	}

#endif

	ATT_SERVER_ReleaseDatabaseAccess();

#if !ENABLE_HIGH_PRIO_DATA_SERVICE
	(void)status;
#endif
}

extern void VXA_bluetoothLE_writeAndNotifyLowPriorityData(uint8_t* buffer) {
	BleStatus status;

	memcpy(highDataRateCharacteristicValue[LOW_PRIORITY_DATA], buffer, 20);

	ATT_SERVER_SecureDatabaseAccess();

	status =
			ATT_SERVER_WriteAttributeValue(
					&highDataRateCharacteristicProperties[LOW_PRIORITY_DATA].characteristicAttribute,
					(const uint8_t *) &highDataRateCharacteristicValue[LOW_PRIORITY_DATA][0],
					20);

#if ENABLE_HIGH_PRIO_DATA_SERVICE
	if (status == BLESTATUS_SUCCESS) /* send notification */
	{
		status =
				ATT_SERVER_SendNotification(
						&highDataRateCharacteristicProperties[1].characteristicAttribute,
						peer);
		//printf("low priority sensor data notification \n");
		printSendNotificationError(status);
	} else {
		assert(status != BLESTATUS_FAILED);

	}

#endif

	ATT_SERVER_ReleaseDatabaseAccess();

#if !ENABLE_HIGH_PRIO_DATA_SERVICE
	(void)status;
#endif
}

extern void VXA_bluetoothLE_init(void) {
	/* return value for BLE stack configuration */
	BleStatus bleInitReturn;

	BLE_notification_t configParams;

	/* return value for BLE task create */
	int8_t bleTaskInitReturn;

	BLE_returnStatus_t _returnValue = BLE_setDeviceName(
			(uint8_t *) VXA_BLE_DEVICE_NAME, strlen(VXA_BLE_DEVICE_NAME));
	assert(_returnValue == BLE_SUCCESS);

	/* enable and register notification callback for bluetooth device connect and disconnect*/
	configParams.callback = bluetoothConnectedNotificationCallback;
	configParams.enableNotification = BLE_ENABLE_NOTIFICATION;

	BLE_enablenotificationForConnect(configParams);

	/* Registering the BLE Services  */
	bleInitReturn = BLE_customServiceRegistry(vxaBleServiceRegister);

	/* Checking data NULL pointer condition */
	assert(bleInitReturn != BLESTATUS_FAILED);
	if (bleInitReturn != BLESTATUS_FAILED) {
		printf("BLE Service registry was failure,due to NULL Pointer \n");
	}

	/* Initialize the whole BLE stack */
	bleInitReturn = BLE_coreStackInit();

	if (BLESTATUS_FAILED == bleInitReturn) {
		assert(0);
		printf("BLE Boot up process Failed,.! \n");
	} else {
		/* Task creating for BTLE stateHandler to establish the connection */
		bleTaskInitReturn = xTaskCreate((xTaskHandle) vxaBleHandler,
				(const char * const ) "BLE", VXA_BLE_STACK_SIZE_FOR_TASK, NULL,
				VXA_BLE_TASK_PRIORITY, &VXA_bleTask);

		/* BLE task creatioon fail case */
		if (pdPASS != bleTaskInitReturn) {
			printf("BLE Task was not created, Due to Insufficient heap memory");
			assert(0);
		} else {
			printf("BLE Task created and stack initialized \n");
		}
	}
}

extern void VXA_bluetoothLE_deinit(void) {
	/*Suspend the BLE task*/
	vTaskSuspend((xTaskHandle) &VXA_bleTask);
}

/** ************************************************************************* */
