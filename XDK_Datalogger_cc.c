

/* constant definitions ***************************************************** */


/* global error flags to set by incorrect sensor init*/
/* inline functions ********************************************************* */

/**
 * @brief This is a template function where the user can write his custom application.
 *
 */
void appInitSystem(xTimerHandle xTimer)
{
    (void) (xTimer);
    /*Call the RHC init API */
    init();
}

/* local init-functions including Systeminit and all Timer******************** */
extern void init(void) {
	Retcode_T sdInitreturnValue;
	SDC_sdCardAppReturn_t csuReturnValue = 0;
	FRESULT fileSystemResult = 0;
	TaskHandle_t xHandle = NULL;
	TaskHandle_t xHandle1 = NULL;
	TaskHandle_t xHandle2 = NULL;
	const char *pcTextForTask1 = "TASK1 IS RUNNING";
	const char *pcTextForTask2 = "TASK2 IS RUNNING";
	const char *pcTextForTask3 = "TASK3 IS RUNNING";
	FILINFO initfno;
	/* Initialize SD card */
	sdInitreturnValue = SDCardDriver_Init();
	ReadSensorSemaphor = xSemaphoreCreateMutex();
	if (ReadSensorSemaphor == NULL )
	{
		assert(0);
	}
	ActiveBuffer = &pingBuffer;
	BackBuffer = &pongBuffer;
	if (sdInitreturnValue !=RETCODE_OK) { /* Debug fail case test SDC Init */
		assert(0);
	} else {
		if (SDC_diskInitStatus != SDCARD_STATUS_INITIALIZED) /*SD-Card Disk Not Initialized */
		{
			SDC_diskInitStatus = SDCardDriver_DiskInitialize(SDC_DRIVE_ZERO); /* Initialize SD card */
		}
	   if (SDCARD_STATUS_INITIALIZED == SDC_diskInitStatus) {
			if (f_mount(&globalmnt,DEFAULT_LOGICAL_DRIVE,FORCE_MOUNT) != FR_OK) {
				assert(0);
			}
		}
	}
	if (SDC_APP_ERR_ERROR == csuReturnValue) { /* Debug fail case test for SDC Read/Write */
		assert(0);
	} else {
		getIniValues(); /**< get Sensorconfiguration*/
		if (FR_OK != f_stat("logger.ini", &initfno)) {
			missingFile = 1;
		}
		if (strcmp(config.fileformat, "custom") == 0) {
			/**< search for custlog.ini on Sd-Card*/
			if (FR_OK == f_stat("custlog.ini", &initfno)) {
				if (Count_CustLogLines() == 2) /**< only read the custlog lines if lines == 2*/
				{
					customLog_LineRead(customHeader, custstring);
				} else {
					PTD_pinOutSet(PTD_PORT_LED_RED, PTD_PIN_LED_RED);
					strcpy(customHeader,
							"custlog.ini not equals specification");
					strcpy(custstring, "custlog.ini not equals specification");
					printf("custlog.ini not equals specification\n");
					exit(0);
				}
			} else {
				strcpy(customHeader, "CUSTLOG INI MISSING");
				strcpy(custstring, "CUSTLOG INI MISSING");
				missingFile = 1;
			}
		} else {
			customHeader[0] = '\0';
			custstring[0] = '\0';
		}
		scan_files(); /**< scan the files on SD-Card*/
		Sensor_init(); /**< Initialize the Sensors*/
		fileSystemResult = f_open(&fileObject, filename,
				FA_OPEN_EXISTING | FA_WRITE);
		if (fileSystemResult != FR_OK) {
			sprintf(filename, config.filename, buttoncount);
			fileSystemResult = f_open(&fileObject, filename,
					FA_CREATE_ALWAYS | FA_WRITE);
			if (fileSystemResult != FR_OK) {
				PTD_pinOutSet(PTD_PORT_LED_RED, PTD_PIN_LED_RED);
				exit(0);
			}
		}
		fileSystemResult = f_lseek(&fileObject, f_size(&fileObject));
		if (fileSystemResult != FR_OK) {
			PTD_pinOutSet(PTD_PORT_LED_RED, PTD_PIN_LED_RED);
		}
		if(xTaskCreate(normal_blink, (const char * const) pcTextForTask2,
				configMINIMAL_STACK_SIZE, NULL, 4, &xHandle1)!=pdTRUE)
				{
					assert(0);
				}
		if(xTaskCreate(SDC_writeBackBuffer, (const char * const) pcTextForTask3,
				configMINIMAL_STACK_SIZE, NULL, 4, &xHandle2)!=pdTRUE)
								{
					assert(0);
				}
		if(xTaskCreate(UpdateSensorValues, (const char * const) pcTextForTask1,
				configMINIMAL_STACK_SIZE, NULL, 4, &xHandle)!=pdTRUE)
								{
					assert(0);
				}
	}
}

/* API documentation is in the configuration header */
void PB0_Init() {
	PTD_pinModeSet (PTD_GET_PORT_PIN_MODE_DOUT(PUSH_BUTTON_1));PTD_intConfig((GPIO_Port_TypeDef) PTD_PORT_PUSH_BUTTON_1, PTD_PIN_PUSH_BUTTON_1,
			0, PB0_INTERRUPT_EDGE_RISING, PB0_INTERRUPT_ENABLE,
			(PTD_intrCallback) &PB0_InterruptCallback);
	PTD_intEnable(PTD_PIN_PUSH_BUTTON_1);
}

extern void Sensor_init(void) {
	uint32_t bytesPerSample = TIMESTAMP_MAXBYTES;
	uint32_t validNumberOfBytes = 0;
	uint32_t samplesPerSecond = 0;
	uint32_t enabledSensorCounter = 0;/**< count the enabled sensor to calculate the json overhead*/

	if (config.bma280_enabled == 1) {
		bma_280_init();
		bytesPerSample += BMA280_MAXBYTES; /**< add the max byte length of the BMA 280 senor parameter*/
		enabledSensorCounter++;
	}

	if (config.bme280_enabled == 1) {
		bme_280_init();
		bytesPerSample += BME280_MAXBYTES; /**< add the max byte length of the BME 280 senor parameter*/
		enabledSensorCounter++;
	}

	if (config.bmg160_enabled == 1) {
		bmg_160_init();
		bytesPerSample += BMG160_MAXBYTES; /**< add the max byte length of the BMG 160 senor parameter*/
		enabledSensorCounter++;
	}

	if (config.bmi160_enabled == 1) {
		bmi_160_init();
		bytesPerSample += BMI160_MAXBYTES; /**< add the max byte length of the BMI 160 senor parameter*/
		enabledSensorCounter += 2;
	}

	if (config.bmm150_enabled == 1) {
		bmm_150_init();
		bytesPerSample += BMM150_MAXBYTES; /**< add the max byte length of the BMM 150 senor parameter*/
		enabledSensorCounter++;
	}

	if (config.max44009_enabled == 1) {
		max_44009_init();
		bytesPerSample += MAX44009_MAXBYTES; /**< add the max byte length of the MAX 44009 senor parameter*/
		enabledSensorCounter++;
	}
	if (strcmp(conf->fileformat, "json") == 0) {
		bytesPerSample += enabledSensorCounter * JSONOVERHEAD_PER_SENSOR;
	}
	validNumberOfBytes = BYTESPERS / bytesPerSample; /**< max number of bytes reffering to BYTESPERS and the samplingrate in ms*/
	samplesPerSecond = (validNumberOfBytes / 10) * 10;

	/**< set the sampling rate according to the rounded value of samples per second*/
	if (samplesPerSecond > 300) {
		fastestSamplingRate = samplesPerSecond / 100 * 100;
	} else if (samplesPerSecond > 100) {
		fastestSamplingRate = samplesPerSecond / 50 * 50;
	} else {
		fastestSamplingRate = samplesPerSecond;
	}
	minimalTicks = 1000 / fastestSamplingRate; /**< set the minimal valid Tickrate*/
	if (minimalTicks < 2) {
		minimalTicks = 2;
	}
	/**< set configuration to minimalTicks*/
	if (config.bma280_sampling_rate > fastestSamplingRate) {
		config.bma280_sampling_rate_timer_ticks = minimalTicks;
		config.bma280_sampling_rate_remaining_ticks = minimalTicks;
	}

	if (config.bmg160_sampling_rate > fastestSamplingRate) {
		config.bmg160_sampling_rate_timer_ticks = minimalTicks;
		config.bmg160_sampling_rate_remaining_ticks = minimalTicks;
	}

	if (config.bmi160_sampling_rate > fastestSamplingRate) {
		config.bmi160_sampling_rate_timer_ticks = minimalTicks;
		config.bmi160_sampling_rate_remaining_ticks = minimalTicks;
	}

	if (config.bmm150_sampling_rate > fastestSamplingRate) {
		config.bmm150_sampling_rate_timer_ticks = minimalTicks;
		config.bmm150_sampling_rate_remaining_ticks = minimalTicks;
	}

	if (config.bme280_sampling_rate > fastestSamplingRate) {
		config.bme280_sampling_rate_timer_ticks = minimalTicks;
		config.bme280_sampling_rate_remaining_ticks = minimalTicks;
	}

	if (config.max44009_sampling_rate > fastestSamplingRate) {
		config.max44009_sampling_rate_timer_ticks = minimalTicks;
		config.max44009_sampling_rate_remaining_ticks = minimalTicks;
	}
	printf("Maximum sampling rate for this configuration: %ld Hz\n",
			fastestSamplingRate);
	printf("Maximum ticks for this configuration: %ld ms\n", minimalTicks);
	printf("BMA280 ticks %ld ms \n", config.bma280_sampling_rate_timer_ticks);
	printf("BMG160 ticks %ld ms\n", config.bmg160_sampling_rate_timer_ticks);
	printf("BMI160 ticks %ld ms\n", config.bmi160_sampling_rate_timer_ticks);
	printf("BMM150 ticks %ld ms\n", config.bmm150_sampling_rate_timer_ticks);
	printf("BME280 ticks %ld ms\n", config.bme280_sampling_rate_timer_ticks);
	printf("MAX ticks %ld ms\n", config.max44009_sampling_rate_timer_ticks);
	PB0_Init();
}

/* End of Init-Section********************************************************************************** */

/* All Callbackfunction for Timer and Interruptevents like buttonpressed********************************* */
/* API documentation is in the configuration header */
void PB0_InterruptCallback() {
	BackBuffer->length = 0;
	BackBuffer->data[0] = 0;
	ActiveBuffer->length = 0;
	ActiveBuffer->data[0] = 0;
	newFile = 1;
	addnewfile = 1;
	closefile = 1;
	buttoncount++;
	sprintf(filename, config.filename, buttoncount);
}

/* API documentation is in the configuration header */
void normal_blink(void *pvParameters) {
	int8_t sdc_status = 0; /**< variable to check the SDC-Status, set if no SD-Card is found*/
	int8_t led_sdc_status = 0; /**< variable to check the led-status by missing SD-Card blininterval*/
	int8_t missingfile_led = 0; /**< variable to check the led-status by missing INI-File blinkinterval*/
	int8_t btnpressed_led = 0; /**< variable to check the led-status by buttonpressed-event*/
	int8_t normal_led_status = 0; /**< variable to check the led-status by normal run-mode*/
	(void) pvParameters;
	for (;;) {
		if (SDCARD_INSERTED == SDCardDriver_GetDetectStatus()) {
			sdc_status = 0;
			PTD_pinOutClear(PTD_PORT_LED_RED, PTD_PIN_LED_RED);
		} else {
			sdc_status = 1;
		}
		if ((normal_led_status == 0) && (missingFile == 0) && (sdc_status == 0)
				&& (addnewfile == 0)) {
			PTD_pinOutSet(PTD_PORT_LED_ORANGE, PTD_PIN_LED_ORANGE);
			normal_led_status = 1;
			static_assert((portTICK_RATE_MS != 0),"Tick rate MS is zero");
		 vTaskDelay((portTickType) 500 / portTICK_RATE_MS);
		} else if ((normal_led_status == 1) && (missingFile == 0)
				&& (sdc_status == 0) && (addnewfile == 0)) {
			PTD_pinOutClear(PTD_PORT_LED_ORANGE, PTD_PIN_LED_ORANGE);
			normal_led_status = 0;
			static_assert((portTICK_RATE_MS != 0),"Tick rate MS is zero");
			vTaskDelay((portTickType) 3000 / portTICK_RATE_MS);
		} else if ((addnewfile == 1) && (btnpressed_led == 0)) {
			PTD_pinOutSet(PTD_PORT_LED_YELLOW, PTD_PIN_LED_YELLOW);
			btnpressed_led = 1;
			static_assert((portTICK_RATE_MS != 0),"Tick rate MS is zero");
			vTaskDelay((portTickType) 2000 / portTICK_RATE_MS);
		} else if ((addnewfile == 1) && (btnpressed_led == 1)) {
			PTD_pinOutClear(PTD_PORT_LED_YELLOW, PTD_PIN_LED_YELLOW);
			btnpressed_led = 0;
			addnewfile = 0;
		} else if ((sdc_status == 1) && (led_sdc_status == 0)) {
			PTD_pinOutSet(PTD_PORT_LED_RED, PTD_PIN_LED_RED);
			led_sdc_status = 1;
			static_assert((portTICK_RATE_MS != 0),"Tick rate MS is zero");
			vTaskDelay((portTickType) 500 / portTICK_RATE_MS);
		} else if ((sdc_status == 1) && (led_sdc_status == 1)) {
			PTD_pinOutClear(PTD_PORT_LED_RED, PTD_PIN_LED_RED);
			led_sdc_status = 0;
			static_assert((portTICK_RATE_MS != 0),"Tick rate MS is zero");
			vTaskDelay((portTickType) 500 / portTICK_RATE_MS);
		} else if ((missingFile == 1) && (missingfile_led == 0)) {
			PTD_pinOutSet(PTD_PORT_LED_RED, PTD_PIN_LED_RED);
			missingfile_led = 1;
			static_assert((portTICK_RATE_MS != 0),"Tick rate MS is zero");
			vTaskDelay((portTickType) 1000 / portTICK_RATE_MS);
		} else if ((missingFile == 1) && (missingfile_led == 1)) {
			PTD_pinOutClear(PTD_PORT_LED_RED, PTD_PIN_LED_RED);
			missingfile_led = 0;
			static_assert((portTICK_RATE_MS != 0),"Tick rate MS is zero");
			vTaskDelay((portTickType) 1000 / portTICK_RATE_MS);
		}
	}
}

/* API documentation is in the configuration header */
void SDC_writeBackBuffer(void *pvParameters) {
	(void) pvParameters;
	TickType_t xLastWakeTimeBufferWrite;
	const TickType_t xBufferWriteFrequency = SD_TASK_INTERVAL_IN_TICKS;
	int suceedwrite = 0;
	int fprintfret = 0;
	FRESULT fileSystemResult = 0;
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTimeBufferWrite = xTaskGetTickCount();
	for (;;) {
		vTaskDelayUntil(&xLastWakeTimeBufferWrite, xBufferWriteFrequency);
		uint32_t Ticks = (portTickType)1;
        if (Ticks != UINT32_MAX) /* Validated for portMAX_DELAY to assist the task to wait Infinitely (without timing out) */
        {
            Ticks /= portTICK_RATE_MS;
        }
		if ((xSemaphoreTake(ReadSensorSemaphor, (portTickType)Ticks)
				== pdTRUE)) {
			if (suceedwrite == 1) {
				BackBuffer->length = 0;
				BackBuffer->data[0] = 0;
				if (ActiveBuffer == &pingBuffer) {
					ActiveBuffer = &pongBuffer;
					BackBuffer = &pingBuffer;
				} else {
					ActiveBuffer = &pingBuffer;
					BackBuffer = &pongBuffer;
				}
				suceedwrite = 0;
			}
			if(xSemaphoreGive(ReadSensorSemaphor)!=pdTRUE)
			{
				assert(0);
			}
		}
		else
		{
			assert(0);
		}
		if (closefile == 0) {
			fprintfret = f_printf(&fileObject, BackBuffer->data);
			if (fprintfret != -1) {
				f_sync(&fileObject);
				suceedwrite = 1;
			}
		} else {
			newFile = 1;
			writeLogFooter(conf, &fileObject);
			BackBuffer->length = 0;
			BackBuffer->data[0] = 0;
			f_close(&fileObject);
			fileSystemResult = f_open(&fileObject, filename,
					FA_CREATE_ALWAYS | FA_WRITE);
			if (fileSystemResult != FR_OK) {
				PTD_pinOutSet(PTD_PORT_LED_ORANGE, PTD_PIN_LED_ORANGE);
			}
			fileSystemResult = f_lseek(&fileObject, f_size(&fileObject));
			if (fileSystemResult != FR_OK) {
				PTD_pinOutSet(PTD_PORT_LED_YELLOW, PTD_PIN_LED_ORANGE);
			}
			closefile = 0;
		}
	}
}

/* API documentation is in the configuration header */
void UpdateSensorValues(void *pvParameters) {
	(void) pvParameters;
	TickType_t xLastWakeTimeSensorRead;
	TickType_t firstSampleTicks = 0;
	uint32_t serialNumber = 0;
	const TickType_t xSensorReadFrequency = SAMPLE_TASK_INTERVAL_IN_TICKS;
	int32_t buffstatus = 0;
	uint8_t valuesToWrite = 0;
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTimeSensorRead = xTaskGetTickCount();
	for (;;) {
		vTaskDelayUntil(&xLastWakeTimeSensorRead, xSensorReadFrequency);
		valuesToWrite = sampleSensors(conf);
		if (valuesToWrite) {
	        uint32_t Ticks = 1;

	        if (Ticks != UINT32_MAX) /* Validated for portMAX_DELAY to assist the task to wait Infinitely (without timing out) */
	        {
	            Ticks /= portTICK_RATE_MS;
	        }
			if ((xSemaphoreTake(ReadSensorSemaphor, Ticks)
					== pdTRUE)) {
				buffstatus = BUFFSIZE - (ActiveBuffer->length + JSONOVERHEAD);
				if (buffstatus > 0) {
					if ((newFile == 1)) {
						firstSampleTicks = xLastWakeTimeSensorRead;
						ActiveBuffer->length = 0;
						ActiveBuffer->data[0] = 0;
						writeLogHeader(conf, customHeader);
						newFile = 0;
						serialNumber = 0;
					}
					writeLogEntry(conf, custstring,
							xLastWakeTimeSensorRead - firstSampleTicks,
							serialNumber);
					serialNumber++;
				}
				if(xSemaphoreGive(ReadSensorSemaphor)!= pdTRUE)
				{
					assert(0);
				}
			}
		}
	}
}

/* End of Callback-Section********************************************************************************** */

/* All functions that be needed for read out logger.ini custlog.ini an the Writeprocess to SD-Card  */
/* API documentation is in the configuration header */

FRESULT scan_files() {
	FRESULT res;
	FILINFO fno;
	DIR dir;
	int i = 0;
	char *fn, *p; /* This function assumes non-Unicode configuration */
	TCHAR newestfile[13] = { 0 }; /**< variable to store the filename of the newest file on SD-Card*/
	char *path = ""; /**< variable to store rootpath to SD-Card*/
	long temp_filenumber = 0;
	long filenumber = 0;

	res = f_opendir(&dir, path); /* Open the directory */
	if (res == FR_OK) {
		i = strlen(path);
		for (;;) {
			res = f_readdir(&dir, &fno); /* Read a directory item */
			if (res != FR_OK || fno.fname[0] == 0)
				break; /* Break on error or end of dir */
			if (fno.fname[0] == '.')
				continue; /* Ignore dot entry */

			fn = fno.fname;
			if (fno.fattrib & AM_DIR) { /* It is a directory */
				sprintf(&path[i], "/%s", fn);
				res = scan_files(path);
				path[i] = 0;
				if (res != FR_OK) {
					break;
				}
			} else { /* It is a file. */
				/**< check for newer timestamp and a valid fileformat*/
				if (((strstr(fno.fname, "CSV") != NULL)
						|| (strstr(fno.fname, "CST") != NULL)
						|| (strstr(fno.fname, "JSN") != NULL))) {
					strcpy(newestfile, fn);
					p = newestfile;
					while (*p) { // While there are more characters to process...

						if (isdigit((int) *p)) { // Upon finding a digit, ...
							temp_filenumber = strtol(p, &p, 10); // Read a number, ...
						} else { // Otherwise, move on to the next character.
							p++;
						}
					}
					if (temp_filenumber >= filenumber) {
						filenumber = temp_filenumber;
						buttoncount = filenumber + 1;
					}
				}
			}
		}
	}
	return res;
}

/* API documentation is in the configuration header */
int getIniValues() {
	FRESULT ret = 0;
	uint8_t k;

	/* get the Sensor-Config-Parameters */
	/* Section [general] */
	ret = ini_gets("general", "filename", 0, config.filename, 13, "logger.ini");
	ret = ini_gets("general", "fileformat", "logger_ini_missing.txt",
			config.fileformat, 7, "logger.ini");
	ret = ini_gets("general", "dataformat", "logger_ini_missing.txt",
			config.dataformat, 5, "logger.ini");

	/* first get all strings for sensorconfig*/
	ret = ini_gets("bma280", "bandwidth", "logger_ini_missing.txt", bma280_bw,
			12, "logger.ini");
	ret = ini_gets("bmg160", "bandwidth", "logger_ini_missing.txt", bmg160_bw,
			12, "logger.ini");
	ret = ini_gets("bmi160", "bandwidth_accel", "logger_ini_missing.txt",
			bmi160_accel_bw, 12, "logger.ini");
	ret = ini_gets("bmi160", "bandwidth_gyro", "logger_ini_missing.txt",
			bmi160_gyro_bw, 12, "logger.ini");
	ret = ini_gets("bmm150", "data_rate", "logger_ini_missing.txt", bmm150_data,
			12, "logger.ini");
	ret = ini_gets("bme280", "oversampling", "logger_ini_missing.txt",
			bme280_os, 4, "logger.ini");
	ret = ini_gets("bme280", "filter_coefficient", "logger_ini_missing.txt",
			bme280_coeff, 4, "logger.ini");
	ret = ini_gets("MAX44009", "integration_time", "logger_ini_missing.txt",
			MAX44009_int, 4, "logger.ini");

	/* bma280 Sensor */
	config.bma280_enabled = ini_getl("bma280", "enabled", 0, "logger.ini");
	config.bma280_sampling_rate = ini_getl("bma280", "sampling_rate", 0,
			"logger.ini");
	config.bma280_sampling_rate_timer_ticks = (1000
			/ (ini_getl("bma280", "sampling_rate", 0, "logger.ini")));
	config.bma280_range = ini_getl("bma280", "range", 0, "logger.ini");
	config.bma280_bandwidth = ini_getl("bma280", "bandwidth", 0, "logger.ini");
	config.bma280_sampling_rate_remaining_ticks =
			config.bma280_sampling_rate_timer_ticks;

	if (fastestSamplingRate < config.bma280_sampling_rate) {
		fastestSamplingRate = config.bma280_sampling_rate;
	}

	fprintf(stdout,
			" config.bma280_enabled: %ld\n config.bma280_bandwidth: %ld\n config.bma280_range: %ld\n config.bma280_sampling_rate_timer_ticks: %ld\n \n",
			config.bma280_enabled, config.bma280_bandwidth, config.bma280_range,
			config.bma280_sampling_rate_timer_ticks);

	/* bmg160 Sensor */
	config.bmg160_enabled = ini_getl("bmg160", "enabled", 0, "logger.ini");
	config.bmg160_sampling_rate = ini_getl("bmg160", "sampling_rate", 0,
			"logger.ini");
	config.bmg160_sampling_rate_timer_ticks = (1000
			/ (ini_getl("bmg160", "sampling_rate", 0, "logger.ini")));
	config.bmg160_bandwidth = ini_getl("bmg160", "bandwidth", 0, "logger.ini");
	config.bmg160_sampling_rate_remaining_ticks =
			config.bmg160_sampling_rate_timer_ticks;
	if (fastestSamplingRate < config.bmg160_sampling_rate) {
		fastestSamplingRate = config.bmg160_sampling_rate;
	}
	fprintf(stdout,
			" config.bmg160_enabled: %ld\n config.bmg160_bandwidth: %ld\n config.bmg160_sampling_rate_timer_ticks: %ld\n \n",
			config.bmg160_enabled, config.bmg160_bandwidth,
			config.bmg160_sampling_rate_timer_ticks);

	/* bmi160 */
	config.bmi160_enabled = ini_getl("bmi160", "enabled", 0, "logger.ini");
	config.bmi160_sampling_rate = ini_getl("bmi160", "sampling_rate", 0,
			"logger.ini");
	config.bmi160_sampling_rate_timer_ticks = (1000
			/ (ini_getl("bmi160", "sampling_rate", 0, "logger.ini")));
	config.bmi160_bandwidth_accel = ini_getl("bmi160", "bandwidth_accel", 0,
			"logger.ini");
	config.bmi160_bandwidth_gyro = ini_getl("bmi160", "bandwidth_gyro", 0,
			"logger.ini");
	config.bmi160_range = ini_getl("bmi160", "range", 0, "logger.ini");
	config.bmi160_sampling_rate_remaining_ticks =
			config.bmi160_sampling_rate_timer_ticks;
	if (fastestSamplingRate < config.bmi160_sampling_rate) {
		fastestSamplingRate = config.bmi160_sampling_rate;
	}
	fprintf(stdout,
			" config.bmi160_enabled: %ld\n config.bmi160_bandwidth_accel: %ld\n config.bmi160_bandwidth_gyro: %ld\n config.bmi160_range: %ld\n config.bmi160_sampling_rate_timer_ticks: %ld\n \n",
			config.bmi160_enabled, config.bmi160_bandwidth_accel,
			config.bmi160_bandwidth_gyro, config.bmi160_range,
			config.bma280_sampling_rate_timer_ticks);

	/* bmm150 Sensor */
	config.bmm150_enabled = ini_getl("bmm150", "enabled", 0, "logger.ini");
	config.bmm150_sampling_rate = ini_getl("bmm150", "sampling_rate", 0,
			"logger.ini");
	config.bmm150_sampling_rate_timer_ticks = (1000
			/ (ini_getl("bmm150", "sampling_rate", 0, "logger.ini")));
	config.bmm150_data_rate = ini_getl("bmm150", "data_rate", 0, "logger.ini");
	config.bmm150_sampling_rate_remaining_ticks =
			config.bmm150_sampling_rate_timer_ticks;
	if (fastestSamplingRate < config.bmm150_sampling_rate) {
		fastestSamplingRate = config.bmm150_sampling_rate;
	}
	fprintf(stdout,
			" config.bmm150_enabled: %ld\n config.bmm150_data_rate: %ld\n config.bmm150_sampling_rate_timer_ticks: %ld\n \n",
			config.bmm150_enabled, config.bmm150_data_rate,
			config.bmm150_sampling_rate_timer_ticks);

	/* bme280 sensor */
	config.bme280_enabled = ini_getl("bme280", "enabled", 0, "logger.ini");
	config.bme280_sampling_rate = ini_getl("bme280", "sampling_rate", 0,
			"logger.ini");
	config.bme280_sampling_rate_timer_ticks = (1000
			/ (ini_getl("bme280", "sampling_rate", 0, "logger.ini")));
	config.bme280_oversampling = ini_getl("bme280", "oversampling", 0,
			"logger.ini");
	config.bme280_filter_coefficient = ini_getl("bme280", "filter_coefficient",
			0, "logger.ini");
	config.bme280_sampling_rate_remaining_ticks =
			config.bme280_sampling_rate_timer_ticks;
	if (fastestSamplingRate < config.bme280_sampling_rate) {
		fastestSamplingRate = config.bme280_sampling_rate;
	}
	fprintf(stdout,
			" config.bme280_enabled: %ld\n config.bme280_oversampling: %ld\n config.bme280_filter_coefficient: %ld\n config.bme280_sampling_rate_timer_ticks: %ld\n \n",
			config.bma280_enabled, config.bme280_oversampling,
			config.bme280_filter_coefficient,
			config.bme280_sampling_rate_timer_ticks);

	/* max4409 sensor */
	config.max44009_enabled = ini_getl("MAX44009", "enabled", 0, "logger.ini");
	config.max44009_sampling_rate = ini_getl("MAX44009", "sampling_rate", 0,
			"logger.ini");
	config.max44009_sampling_rate_timer_ticks = (1000
			/ (ini_getl("MAX44009", "sampling_rate", 0, "logger.ini")));
	config.max44009_integration_time = ini_getl("MAX44009", "integration_time",
			0, "logger.ini");
	config.max44009_sampling_rate_remaining_ticks =
			config.max44009_sampling_rate_timer_ticks;
	if (fastestSamplingRate < config.max44009_sampling_rate) {
		fastestSamplingRate = config.max44009_sampling_rate;
	}
	fprintf(stdout,
			" config.max44009_enabled: %ld\n config.max44009_integration_time: %ld\n config.max44009_sampling_rate_timer_ticks: %ld\n \n",
			config.max44009_enabled, config.max44009_integration_time,
			config.max44009_sampling_rate_timer_ticks);

	/*copy filename from config to processvariable filename */
	for (k = 0; k < 13; k++) {
		filename[k] = config.filename[k];
	}
	if (strlen(filename) == 0) {
		PTD_pinOutSet(PTD_PORT_LED_RED, PTD_PIN_LED_RED);
		printf("bad filename in logger.ini\n");
		exit(0);
	}
	return ret;
}

/* API documentation is in the configuration header */
int Count_CustLogLines() {
	TCHAR CharBuffer[256] = { 0 };
	FIL FileObject;
	int lines = 0; /**< variable to store the amount of lines in custlog.ini**/

	if (f_open(&FileObject, "custlog.ini", FA_OPEN_EXISTING | FA_READ)
			== FR_OK) {
		while ((f_eof(&FileObject) == 0)) /**< get line by line until EOF*/
		{
			f_gets((char*) CharBuffer, sizeof(CharBuffer) / sizeof(TCHAR),
					&FileObject);
			lines++; /**< increase lines on every line*/
		}
		f_close(&FileObject);
	} else {
		PTD_pinOutSet(PTD_PORT_LED_RED, PTD_PIN_LED_RED);
	}
	return lines;
}

/* API documentation is in the configuration header */
int customLog_LineRead(TCHAR customHeader[], TCHAR custstring[]) {

	FIL cntObject; /* File objects */
	int l;
	int retval = 0;

	if (f_open(&cntObject, "custlog.ini", FA_OPEN_EXISTING | FA_READ)
			== FR_OK) {
		for (l = 0; (f_eof(&cntObject) == 0); l++) {
			if (l == 0) /* firts line*/
			{
				f_gets(customHeader, CUSTLOGBUFSIZE, &cntObject); /**< get first line*/
			}
			if (l == 1) /* second line*/
			{
				f_gets(custstring, CUSTLOGBUFSIZE, &cntObject);/**< get second line*/
			}
		}
		f_close(&cntObject);
	} else {
		PTD_pinOutSet(PTD_PORT_LED_RED, PTD_PIN_LED_RED);
		retval = 1;
	}
	return retval;
}

/* API documentation is in the configuration header */
uint8_t sampleSensors(configuration *conf) {
	uint8_t writeValues = 0;
	/* countdown the the sampling_rate_remaining_ticks of every enabled Sensor
	 * the sampling_rate_remaining_ticks according to the sampling_rate_timer_ticks which gets a value from 1 to 1000*/
	if (conf->bma280_enabled == 1) {
		conf->bma280_sampling_rate_remaining_ticks--;
		if (conf->bma280_sampling_rate_remaining_ticks < 1) {
			conf->bma280_sampling_rate_remaining_ticks =
					conf->bma280_sampling_rate_timer_ticks;
			bma280_getSensorValues (NULL);
			writeValues = 1;
		}
	}

	if (conf->bmg160_enabled == 1) {
		conf->bmg160_sampling_rate_remaining_ticks--;
		if (conf->bmg160_sampling_rate_remaining_ticks < 1) {
			conf->bmg160_sampling_rate_remaining_ticks =
					conf->bmg160_sampling_rate_timer_ticks;
			bmg160_getSensorValues (NULL);
			writeValues = 1;
		}
	}

	if (conf->bmi160_enabled == 1) {
		conf->bmi160_sampling_rate_remaining_ticks--;
		if (conf->bmi160_sampling_rate_remaining_ticks < 1) {
			conf->bmi160_sampling_rate_remaining_ticks =
					conf->bmi160_sampling_rate_timer_ticks;
			bmi160_getSensorValues (NULL);
			writeValues = 1;
		}
	}

	if (conf->bmm150_enabled == 1) {
		conf->bmm150_sampling_rate_remaining_ticks--;
		if (conf->bmm150_sampling_rate_remaining_ticks < 1) {
			conf->bmm150_sampling_rate_remaining_ticks =
					conf->bmm150_sampling_rate_timer_ticks;
			bmm150_getSensorValues (NULL);
			writeValues = 1;
		}
	}

	if (conf->bme280_enabled == 1) {
		conf->bme280_sampling_rate_remaining_ticks--;
		if (conf->bme280_sampling_rate_remaining_ticks < 1) {
			conf->bme280_sampling_rate_remaining_ticks =
					conf->bme280_sampling_rate_timer_ticks;
			bme280_getSensorValues (NULL);
			writeValues = 1;
		}
	}

	if (conf->max44009_enabled == 1) {
		conf->max44009_sampling_rate_remaining_ticks--;
		if (conf->max44009_sampling_rate_remaining_ticks < 1) {
			conf->max44009_sampling_rate_remaining_ticks =
					conf->max44009_sampling_rate_timer_ticks;
			max44009_getSensorValues (NULL);
			writeValues = 1;
		}
	}
	return writeValues;
}

/* API documentation is in the configuration header */
char * stringReplace(char *search, char *replace, char *string) {
	char *searchStart;
	char tempString[STRINGREPLACEBUFFER] = { 0 };
	int len = 0;
	uint32_t remainingBufSize = STRINGREPLACEBUFFER - strlen(string);
	/** check for search string
	 */
	searchStart = strstr(string, search);
	if (searchStart == NULL) {
		return string;
	}
	/** temporary copy of the string
	 */
	if (remainingBufSize >= strlen(replace)) {
		strcpy((char *) tempString, string);

		/** set first section of the string
		 */
		len = searchStart - string;
		string[len] = '\0';
		/** append second section of the string
		 */
		strcat(string, replace);

		/** append third section of the string
		 */
		len += strlen(search);
		strcat(string, (char*) tempString + len);
		return string;
	} else {
		return NULL;
	}
}

/* API documentation is in the configuration header */
void writeSensorDataCsvHeader(configuration *conf) {
	if (strcmp(conf->dataformat, "raw") == 0) {
		ActiveBuffer->length += sprintf(
				ActiveBuffer->data + ActiveBuffer->length,
				" raw--timestamp[ms]");

		if (conf->bma280_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length,
					";bma280_x;bma280_y;bma280_z");
		}

		if (conf->bmg160_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length,
					";bmg160_x;bmg160_y;bmg160_z");
		}

		if (conf->bmi160_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length,
					";bmi160_a_x;bmi160_a_y;bmi160_a_z");
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length,
					";bmi160_g_x;bmi160_g_y;bmi160_g_z");
		}

		if (conf->bmm150_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length,
					";bmm150_x;bmm150_y;bmm150_z;bmm150_res");
		}

		if (conf->bme280_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length,
					";bme280_temp;bme280_press;bme280_hum");
		}

		if (conf->max44009_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length,
					";max44009_bright");
		}
	} else {
		ActiveBuffer->length += sprintf(
				ActiveBuffer->data + ActiveBuffer->length,
				" unit--timestamp[ms]");

		if (conf->bma280_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length,
					";bma280_x[mg];bma280_y[mg];bma280_z[mg]");
		}

		if (conf->bmg160_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length,
					";bmg160_x[mDeg];bmg160_y[mDeg];bmg160_z[mDeg]");
		}

		if (conf->bmi160_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length,
					";bmi160_a_x[mg];bmi160_a_y[mg];bmi160_a_z[mg]");
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length,
					";bmi160_g_x[mDeg];bmi160_g_y[mDeg];bmi160_g_z[mDeg]");
		}

		if (conf->bmm150_enabled == 1) {
			ActiveBuffer->length +=
					sprintf(ActiveBuffer->data + ActiveBuffer->length,
							";bmm150_x[microT];bmm150_y[microT];bmm150_z[microT];bmm150_res");
		}

		if (conf->bme280_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length,
					";bme280_temp[mDeg];bme280_press[Pa];bme280_hum[rh]");
		}

		if (conf->max44009_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length,
					";max44009_bright[mLux]");
		}
	}
	ActiveBuffer->length += sprintf(ActiveBuffer->data + ActiveBuffer->length,
			";\n");
}

/* API documentation is in the configuration header */
void writeSensorDataCsv(uint64_t timestamp, configuration *conf) {
	/* First check configured dataformat then write the data of all enabled sensors*/
	ActiveBuffer->length += sprintf(ActiveBuffer->data + ActiveBuffer->length,
			"%llu", timestamp);
	if (strcmp(conf->dataformat, "raw") == 0) {
		if (conf->bma280_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length, ";%ld;%ld;%ld",
					getAccelDataRaw.xAxisData, getAccelDataRaw.yAxisData,
					getAccelDataRaw.zAxisData);
		}

		if (conf->bmg160_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length, ";%ld;%ld;%ld",
					getRawData.xAxisData, getRawData.yAxisData, getRawData.zAxisData);
		}
		if (conf->bmi160_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length, ";%ld;%ld;%ld",
					getAccelDataRaw160.xAxisData, getAccelDataRaw160.yAxisData,
					getAccelDataRaw160.zAxisData);
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length, ";%ld;%ld;%ld",
					getGyroDataRaw160.xAxisData, getGyroDataRaw160.yAxisData,
					getGyroDataRaw160.zAxisData);
		}

		if (conf->bmm150_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length,
					";%ld;%ld;%ld;%d", getMagDataRaw.xAxisData,
					getMagDataRaw.yAxisData, getMagDataRaw.zAxisData,
					getMagDataRaw.resistance);
		}

		if (conf->bme280_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length, ";%ld;%ld;%ld",
					bme280lsb.humidity, bme280lsb.pressure,
					bme280lsb.temperature);
		}

		if (conf->max44009_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length, ";%d",
					luxRawData);
		}
	} else {
		if (conf->bma280_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length, ";%ld;%ld;%ld",
					getAccelDataUnit.xAxisData, getAccelDataUnit.yAxisData,
					getAccelDataUnit.zAxisData);
		}

		if (conf->bmg160_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length, ";%ld;%ld;%ld",
					getMdegData.xAxisData, getMdegData.yAxisData, getMdegData.zAxisData);
		}

		if (conf->bmi160_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length, ";%ld;%ld;%ld",
					getAccelDataUnit160.xAxisData,
					getAccelDataUnit160.yAxisData,
					getAccelDataUnit160.zAxisData);
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length, ";%ld;%ld;%ld",
					getGyroDataConv160.xAxisData, getGyroDataConv160.yAxisData,
					getGyroDataConv160.zAxisData);
		}

		if (conf->bmm150_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length,
					";%ld;%ld;%ld;%d", getMagDataUnit.xAxisData,
					getMagDataUnit.yAxisData, getMagDataUnit.zAxisData,
					getMagDataUnit.resistance);
		}

		if (conf->bme280_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length, ";%ld;%ld;%ld",
					bme280s.temperature, bme280s.pressure, bme280s.humidity);
		}

		if (conf->max44009_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length, ";%ld",
					milliLuxData);
		}
	}
	ActiveBuffer->length += sprintf(ActiveBuffer->data + ActiveBuffer->length,
			";\n");
}

/* API documentation is in the configuration header */
void writeSensorDataJson(uint64_t timestamp, uint64_t serialnumber,
		configuration *conf) {
	/* First check configured dataformat then write the data of all enabled sensors*/
	if (strcmp(conf->dataformat, "raw") == 0) {
		ActiveBuffer->length += sprintf(
				ActiveBuffer->data + ActiveBuffer->length, "\"%lld\":{\n",
				serialnumber);
		ActiveBuffer->length += sprintf(
				ActiveBuffer->data + ActiveBuffer->length,
				"\"Timestamp[ms]\":%lld,\n", timestamp);

		if (conf->bma280_enabled == 1) {
			ActiveBuffer->length +=
					sprintf(ActiveBuffer->data + ActiveBuffer->length,
							"\"bma280_x\":%ld,\n" "\"bma280_y\":%ld,\n" "\"bma280_z\":%ld,\n",
							getAccelDataRaw.xAxisData,
							getAccelDataRaw.yAxisData,
							getAccelDataRaw.zAxisData);
		}

		if (conf->bmg160_enabled == 1) {
			ActiveBuffer->length +=
					sprintf(ActiveBuffer->data + ActiveBuffer->length,
							"\"bmg160_x\":%ld,\n" "\"bmg160_y\":%ld,\n" "\"bmg160_z\":%ld,\n",
							getRawData.xAxisData, getRawData.yAxisData,
							getRawData.zAxisData);
		}

		if (conf->bmi160_enabled == 1) {
			ActiveBuffer->length +=
					sprintf(ActiveBuffer->data + ActiveBuffer->length,
							"\"bmi160_a_x\":%ld,\n" "\"bmi160_a_y\":%ld,\n" "\"bmi160_a_z\":%ld,\n" "\"bmi160_g_x\":%ld,\n" "\"bmi160_g_y\":%ld,\n" "\"bmi160_g_z\":%ld,\n",
							getAccelDataRaw160.xAxisData,
							getAccelDataRaw160.yAxisData,
							getAccelDataRaw160.zAxisData,
							getGyroDataRaw160.xAxisData, getGyroDataRaw160.yAxisData,
							getGyroDataRaw160.zAxisData);
		}

		if (conf->bmm150_enabled == 1) {
			ActiveBuffer->length +=
					sprintf(ActiveBuffer->data + ActiveBuffer->length,
							"\"bmm150_x\":%ld,\n" "\"bmm150_y\":%ld,\n" "\"bmm150_z\":%ld,\n",
							getMagDataRaw.xAxisData, getMagDataRaw.yAxisData,
							getMagDataRaw.zAxisData);
		}

		if (conf->bme280_enabled == 1) {
			ActiveBuffer->length +=
					sprintf(ActiveBuffer->data + ActiveBuffer->length,
							"\"bme280_hum\":%ld,\n" "\"bme280_press\":%ld,\n" "\"bme280_temp\":%ld,\n",
							bme280lsb.humidity, bme280lsb.pressure,
							bme280lsb.temperature);
		}

		if (conf->max44009_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length,
					"\"max44009_bright\":%d,\n", luxRawData);
		}
	} else {
		ActiveBuffer->length += sprintf(
				ActiveBuffer->data + ActiveBuffer->length, "\"%lld\":{\n",
				serialnumber);
		ActiveBuffer->length += sprintf(
				ActiveBuffer->data + ActiveBuffer->length,
				"\"Timestamp[ms]\":%lld,\n", timestamp);

		if (conf->bma280_enabled == 1) {
			ActiveBuffer->length +=
					sprintf(ActiveBuffer->data + ActiveBuffer->length,
							"\"bma280_x[mg]\":%ld,\n" "\"bma280_y[mg]\":%ld,\n" "\"bma280_z[mg]\":%ld,\n",
							getAccelDataUnit.xAxisData,
							getAccelDataUnit.yAxisData,
							getAccelDataUnit.zAxisData);
		}

		if (conf->bmg160_enabled == 1) {
			ActiveBuffer->length +=
					sprintf(ActiveBuffer->data + ActiveBuffer->length,
							"\"bmg160_x[mDeg]\":%ld,\n" "\"bmg160_y[mDeg]\":%ld,\n" "\"bmg160_z[mDeg]\":%ld,\n",
							getMdegData.xAxisData, getMdegData.yAxisData,
							getMdegData.zAxisData);
		}

		if (conf->bmi160_enabled == 1) {
			ActiveBuffer->length +=
					sprintf(ActiveBuffer->data + ActiveBuffer->length,
							"\"bmi160_a_x[mg]\":%ld,\n" "\"bmi160_a_y[mg]\":%ld,\n" "\"bmi160_a_z[mg]\":%ld,\n" "\"bmi160_g_x[mDeg]\":%ld,\n" "\"bmi160_g_y[mDeg]\":%ld,\n" "\"bmi160_g_z[mDeg]\":%ld,\n",
							getAccelDataUnit160.xAxisData,
							getAccelDataUnit160.yAxisData,
							getAccelDataUnit160.zAxisData,
							getGyroDataConv160.xAxisData, getGyroDataConv160.yAxisData,
							getGyroDataConv160.zAxisData);
		}

		if (conf->bmm150_enabled == 1) {
			ActiveBuffer->length +=
					sprintf(ActiveBuffer->data + ActiveBuffer->length,
							"\"bmm150_x[microT]\":%ld,\n" "\"bmm150_y[microT]\":%ld,\n" "\"bmm150_z[microT]\":%ld,\n",
							getMagDataUnit.xAxisData, getMagDataUnit.yAxisData,
							getMagDataUnit.zAxisData);
		}

		if (conf->bme280_enabled == 1) {
			ActiveBuffer->length +=
					sprintf(ActiveBuffer->data + ActiveBuffer->length,
							"\"bme280_hum[rh]\":%ld,\n" "\"bme280_press[Pa]\":%ld,\n" "\"bme280_temp[mDeg]\":%ld,\n",
							bme280s.humidity, bme280s.pressure,
							bme280s.temperature);
		}

		if (conf->max44009_enabled == 1) {
			ActiveBuffer->length += sprintf(
					ActiveBuffer->data + ActiveBuffer->length,
					"\"max44009_bright[mLux]\":%ld,\n",
					milliLuxData);
		}
	}
	ActiveBuffer->length += sprintf(ActiveBuffer->data + ActiveBuffer->length,
			"}\n");
}

/* API documentation is in the configuration header */
void writeSensorDataJsonHeader(configuration *conf) {
	/*The Header for Json-Files is called after File is created*/
	if (strcmp(conf->dataformat, "raw") == 0) {
		ActiveBuffer->length += sprintf(
				ActiveBuffer->data + ActiveBuffer->length,
				"SensorValuesRaw:[\n");
	} else {
		ActiveBuffer->length += sprintf(
				ActiveBuffer->data + ActiveBuffer->length,
				"SensorValuesUnit:[\n");
	}
}

/* API documentation is in the configuration header */
void writeSensorDataJsonFooter(FIL *fileObject) {
	/*The Footer for Json-Files is called when fileprocess is closed*/
	f_printf(fileObject, "]");
}

/* API documentation is in the configuration header */
void writeSensorDataCustom(char *customstring, uint64_t timestamp,
		configuration *conf) {
	char buffer[10] = "";
	/*Write the Sensordate as specified in the custlog.ini*/
	char tempCustomString[STRINGREPLACEBUFFER] = { 0 };
	strcpy(tempCustomString, customstring);
	itoa(timestamp, buffer);
	stringReplace("%timestamp", buffer, tempCustomString); /* replace the string "%timestamp" with value of buffer*/
	if (strcmp(conf->dataformat, "raw")) {
		if (conf->bma280_enabled == 1) {
			itoa(getAccelDataRaw.xAxisData, buffer);
			stringReplace("%bma280_x", buffer, tempCustomString);
			itoa(getAccelDataRaw.yAxisData, buffer);
			stringReplace("%bma280_y", buffer, tempCustomString);
			itoa(getAccelDataRaw.zAxisData, buffer);
			stringReplace("%bma280_z", buffer, tempCustomString);
		}

		if (conf->bmg160_enabled == 1) {
			itoa(getRawData.xAxisData, buffer);
			stringReplace("%bmg160_x", buffer, tempCustomString);
			itoa(getRawData.yAxisData, buffer);
			stringReplace("%bmg160_y", buffer, tempCustomString);
			itoa(getRawData.zAxisData, buffer);
			stringReplace("%bmg160_z", buffer, tempCustomString);
		}

		if (conf->bmi160_enabled == 1) {
			itoa(getAccelDataRaw160.xAxisData, buffer);
			stringReplace("%bmi160_a_x", buffer, tempCustomString);
			itoa(getAccelDataRaw160.yAxisData, buffer);
			stringReplace("%bmi160_a_y", buffer, tempCustomString);
			itoa(getAccelDataRaw160.zAxisData, buffer);
			stringReplace("%bmi160_a_z", buffer, tempCustomString);
			itoa(getGyroDataRaw160.xAxisData, buffer);
			stringReplace("%bmi160_g_x", buffer, tempCustomString);
			itoa(getGyroDataRaw160.yAxisData, buffer);
			stringReplace("%bmi160_g_y", buffer, tempCustomString);
			itoa(getGyroDataRaw160.zAxisData, buffer);
			stringReplace("%bmi160_g_z", buffer, tempCustomString);
		}

		if (conf->bmm150_enabled == 1) {
			itoa(getMagDataRaw.xAxisData, buffer);
			stringReplace("%bmm150_x", buffer, tempCustomString);
			itoa(getMagDataRaw.yAxisData, buffer);
			stringReplace("%bmm150_y", buffer, tempCustomString);
			itoa(getMagDataRaw.zAxisData, buffer);
			stringReplace("%bmm150_z", buffer, tempCustomString);
		}

		if (conf->bme280_enabled == 1) {
			itoa(bme280lsb.humidity, buffer);
			stringReplace("%bme280_h", buffer, tempCustomString);
			itoa(bme280lsb.pressure, buffer);
			stringReplace("%bme280_p", buffer, tempCustomString);
			itoa(bme280lsb.temperature, buffer);
			stringReplace("%bme280_t", buffer, tempCustomString);
		}

		if (conf->max44009_enabled == 1) {
			itoa(luxRawData, buffer);
			stringReplace("%max44009_bright", buffer, tempCustomString);
		}
	} else {
		if (conf->bma280_enabled == 1) {
			itoa(getAccelDataUnit.xAxisData, buffer);
			stringReplace("%bma280_x", buffer, tempCustomString);
			itoa(getAccelDataUnit.yAxisData, buffer);
			stringReplace("%bma280_y", buffer, tempCustomString);
			itoa(getAccelDataUnit.zAxisData, buffer);
			stringReplace("%bma280_z", buffer, tempCustomString);
		}

		if (conf->bmg160_enabled == 1) {
			itoa(getMdegData.xAxisData, buffer);
			stringReplace("%bmg160_x", buffer, tempCustomString);
			itoa(getMdegData.yAxisData, buffer);
			stringReplace("%bmg160_y", buffer, tempCustomString);
			itoa(getMdegData.zAxisData, buffer);
			stringReplace("%bmg160_z", buffer, tempCustomString);
		}

		if (conf->bmi160_enabled == 1) {
			itoa(getAccelDataUnit160.xAxisData, buffer);
			stringReplace("%bmi160_a_x", buffer, tempCustomString);
			itoa(getAccelDataUnit160.yAxisData, buffer);
			stringReplace("%bmi160_a_y", buffer, tempCustomString);
			itoa(getAccelDataUnit160.zAxisData, buffer);
			stringReplace("%bmi160_a_z", buffer, tempCustomString);
			itoa(getGyroDataConv160.xAxisData, buffer);
			stringReplace("%bmi160_g_x", buffer, tempCustomString);
			itoa(getGyroDataConv160.yAxisData, buffer);
			stringReplace("%bmi160_g_y", buffer, tempCustomString);
			itoa(getGyroDataConv160.zAxisData, buffer);
			stringReplace("%bmi160_g_z", buffer, tempCustomString);
		}

		if (conf->bmm150_enabled == 1) {
			itoa(getMagDataUnit.xAxisData, buffer);
			stringReplace("%bmm150_x", buffer, tempCustomString);
			itoa(getMagDataUnit.yAxisData, buffer);
			stringReplace("%bmm150_y", buffer, tempCustomString);
			itoa(getMagDataUnit.zAxisData, buffer);
			stringReplace("%bmm150_z", buffer, tempCustomString);
		}

		if (conf->bme280_enabled == 1) {
			itoa(bme280s.humidity, buffer);
			stringReplace("%bme280_h", buffer, tempCustomString);
			itoa(bme280s.pressure, buffer);
			stringReplace("%bme280_p", buffer, tempCustomString);
			itoa(bme280s.temperature, buffer);
			stringReplace("%bme280_t", buffer, tempCustomString);
		}

		if (conf->max44009_enabled == 1) {
			itoa(milliLuxData, buffer);
			stringReplace("%max44009_bright", buffer, tempCustomString);
		}
	}
	ActiveBuffer->length += sprintf(ActiveBuffer->data + ActiveBuffer->length,
			"%s\n", tempCustomString);
	memset(tempCustomString, 0, STRINGREPLACEBUFFER);
}

/* API documentation is in the configuration header */
void writeLogEntry(configuration *conf, char *customstring,
		uint32_t currentTicks, uint32_t serialnumber) {
	static uint32_t numberOfTicks = TIMESTAMP_UNIT_IN_TICKS;
	uint32_t timestamp = 0;

	timestamp = ((currentTicks) / numberOfTicks); /*calculate the timestamp without the logStartTime-Offset*/
	/*check the configured fileformat an call the related write-function */
	if (strcmp(conf->fileformat, "csv") == 0) {
		writeSensorDataCsv(timestamp, conf);
	} else if (strcmp(conf->fileformat, "json") == 0) {
		writeSensorDataJson(timestamp, serialnumber, conf);
	} else if (strcmp(conf->fileformat, "custom") == 0) {
		writeSensorDataCustom(customstring, timestamp, conf);
	} else {
		printf("Error writing log entry: no valid file format found\n");
	}
}

/* API documentation is in the configuration header */
void writeLogHeader(configuration *conf, char *customheader) {
	logActive = 0;

	if (strcmp(conf->fileformat, "csv") == 0) {
		writeSensorDataCsvHeader(conf);
	} else if (strcmp(conf->fileformat, "json") == 0) {
		writeSensorDataJsonHeader(conf);
	} else if (strcmp(conf->fileformat, "custom") == 0) {
		ActiveBuffer->length += sprintf(
				ActiveBuffer->data + ActiveBuffer->length, "%s\n",
				customheader);
	} else {
		ActiveBuffer->length += sprintf(
				ActiveBuffer->data + ActiveBuffer->length, "%s\n", "false");
	}
}

/* API documentation is in the configuration header */
void writeLogFooter(configuration *conf, FIL *fileObject) {
	logActive = 0;

	if ((strcmp(conf->fileformat, "csv") == 0)
			|| (strcmp(conf->fileformat, "custom") == 0)) {
		/**nothing to do since no footer*/
	} else if (strcmp(conf->fileformat, "json") == 0) {
		writeSensorDataJsonFooter(fileObject);
	} else {
		printf("Error writing log footer: no valid file format found\n");
	}
}

/* End of user-Section********************************************************************************** */

/**
 *  @brief API to Deinitialize the PSD module
 */
extern void PSD_deinit(void) {
	bma_280_deInit();
	max_44009_deInit();
	bme_280_deInit();
	bmg_160_deInit();
	bmi160_deInit();
	bmm_150_deInit();
}

/** ************************************************************************* */
