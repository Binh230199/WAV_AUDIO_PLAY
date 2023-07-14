/* --COPYRIGHT--, Hoang Van Binh
 *
 ******************************SD Card**********************************
 *  Description: Process files on SD Card
 *
 *  Version:  	1.0
 *  Author: 	Hoang Van Binh
 *  Release: 	Sep 27, 2021
 *  Built with STM32CubeIDE version 1.7.0
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <sd.h>
/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart2;

//---------------------------------------------------------------------//
/* Exported functions ---------------------------------------------------------*/

/**
 * @brief  Print data using UART.
 * @param  string: data want to print
 */
void print(char *string) {
#ifdef DEBUG_INFO
	uint16_t len = strlen(string);
	HAL_UART_Transmit(&DEBUG_INFO, (uint8_t *) string, len, HAL_MAX_DELAY);
#endif
}

/**
 * @brief  Get total space information of SD Card.
 */
uint32_t SD_GetTotalSpace(void)
{
	static FATFS *fs; //file system
	static DWORD freClust, totalSpace;
	static FRESULT res;

	res = f_getfree("", &freClust, &fs);
	if(res == FR_OK)
	{
		totalSpace = (fs -> n_fatent - 2) * fs -> csize * 0.5;
		char tmsg [40];
		sprintf(tmsg, "Total memory space: %lu Kbytes\r\n\r\n", totalSpace);
		print(tmsg);
	}
	else
		print("Memory determination is failed!\r\n\r\n");
	return res;
}

/**
 * @brief  Get free space information of SD Card.
 */
uint32_t SD_GetFreeSpace(void)
{
	static FATFS *fs; //file system
	static DWORD freClust, freeSpace;
	static FRESULT res;

	res = f_getfree("", &freClust, &fs);
	if(res == FR_OK)
	{
		freeSpace = freClust * fs -> csize * 0.5;
		char tmsg [40];
		sprintf(tmsg, "Free memory space: %lu Kbytes\r\n\r\n", freeSpace);
		print(tmsg);
	}
	else
		print("Memory determination is failed!\r\n\r\n");
	return res;
}

/**
 * @brief  Mount SD Card.
 * @param  path: directory
 */
FRESULT SD_Mount(const TCHAR* path)
{
	static FATFS fatfs; //file system
	static FRESULT res;

	res = f_mount(&fatfs, path, 0);
	if(res == FR_OK)
		print("Mount SD Card is successful!\r\n\r\n");
	else
		print("Mount SD Card is error!\r\n\r\n");
	return res;
}
/**
 * @brief  Unmount SD Card.
 * @param  path: directory
 */
FRESULT SD_Unmount(const TCHAR* path)
{
	static FRESULT res;
	res = f_mount(NULL, path, 1);
	if(res == FR_OK)
		print("Unmount SD Card is successful!\r\n\r\n");
	else
		print("Unmount SD Card is failed!\r\n\r\n");
	return res;
}

/**
 * @brief  Scan file and directory.
 * @param  Path: directory
 * @retval File function return code
 */
FRESULT SD_Scan(char* Path)
{
	static DIR dir;
	static UINT i;
	static FRESULT res;
	static FILINFO fno;

	char *path = malloc(80*sizeof(char));
	sprintf(path, "%s", Path);
	//open the directory
	res = f_opendir(&dir, path);
	if(res == FR_OK)
	{
		while(1)
		{
			//read a directory item
			res = f_readdir(&dir, &fno);
			//break on error or end of dir
			if(res != FR_OK || fno.fname[0] == 0) break;
			//it is a directory
			if(fno.fattrib & AM_DIR)
			{
				if(!(strcmp("SYSTEM~1", fno.fname))) continue;
				if(!(strcmp("System Volume Information", fno.fname))) continue;
				char *buf = malloc(50*sizeof(char));
				sprintf(buf, "DIR: %s\r\n", fno.fname);
				print(buf);
				free(buf);
				i = strlen(path);
				sprintf(&path[i], "/%s", fno.fname);
				//enter the directory
				res = SD_Scan(path);
				if(res != FR_OK) break;
				path[i] = 0;
			}
			else
			{	//it is a file
				char *buf = malloc(50*sizeof(char));
				sprintf(buf, "File: %s/%s\n", path, fno.fname);
				print(buf);
			}
		}
		f_closedir(&dir);
	}
	free(path);
	print("\r\n");
	return res;
}

/**
 * @brief  Create a new file.
 * @param  name: name of file
 * @retval File function return code
 */
FRESULT SD_CreateFile(char *name)
{
	static FIL 	fil; //file
	static FRESULT res; //
	static FILINFO fno;

	res = f_stat(name, &fno);
	if(res == FR_OK)
	{	//file is existing
		char *buf = malloc(60*sizeof(char));
		sprintf(buf, "Error! File -%s- already exist!\r\n\r\n", name) ;
		print(buf);
		free(buf);
		return res;
	}
	else
	{	//File not created yet
		//Create a file
		res = f_open(&fil, name, FA_CREATE_ALWAYS | FA_WRITE);
		if(res != FR_OK)
		{	//Error
			char *buf = malloc(80*sizeof(char));
			sprintf(buf, "Error %d while creating file -%s-!\r\n\r\n", res, name);
			print(buf);
			free(buf);
			return res;
		}
		else
		{	//Successful
			char *buf = malloc(80*sizeof(char));
			sprintf(buf, "File -%s- was created successfully.\r\n\r\n", name);
			print(buf);
			free(buf);
		}
		//Close file
		res = f_close(&fil);
		if(res != FR_OK)
		{	//Successful
			char *buf = malloc(80*sizeof(char));
			sprintf(buf, "Error %d while closing file -%s-!\r\n\r\n", res, name);
			print(buf);
			free(buf);
		}
	}
	return res;
}

/**
 * @brief  Write data into the file.
 * @param  name: name of file
 * @param  data: data want to be writed
 * @retval File function return code
 */
FRESULT SD_WriteFile(char *name, char *data, uint32_t size)
{
	static FIL 	fil; //file
	static FRESULT res; //
	static UINT bw; //read/write file pointer variables
	static FILINFO fno; //file information variable

	res = f_stat(name, &fno);
	if(res != FR_OK)
	{	//File not existed
		char *buf = malloc(80*sizeof(char));
		sprintf(buf, "Error! File -%s- is not existing to write!\r\n\r\n", name);
		print(buf);
		free(buf);
		return res;
	}
	else
	{	//File is existing
		//Open file to write data
		res = f_open(&fil, name, FA_OPEN_EXISTING | FA_WRITE);
		if(res != FR_OK)
		{	//Error
			char *buf = malloc(80*sizeof(char));
			sprintf(buf, "Error %d while opening file -%s-!\r\n\r\n", res, name);
			print(buf);
			free(buf);
			return res;
		}
		else
		{	//Successful
			//Write data
			res = f_write(&fil, data, size, &bw);
			if(res != FR_OK)
			{	//Error
				char *buf = malloc(80*sizeof(char));
				sprintf(buf, "Error %d while writing data into file -%s-!\r\n\r\n", res, name);
				print(buf);
				free(buf);
			}
			else
			{
				char *buf = malloc(80*sizeof(char));
				sprintf(buf, "File -%s- was wrote successfully.\r\n\r\n", name);
				print(buf);
				free(buf);
			}
			//Close file
			res = f_close(&fil);
			if(res != FR_OK)
			{	//Error
				char *buf = malloc(80*sizeof(char));
				sprintf(buf, "Error %d while closing file -%s-!\r\n\r\n", res, name);
				print(buf);
				free(buf);
			}
		}
	}
	return res;
}

/**
 * @brief  Read data from the file.
 * @param  name: name of file
 * @param  pData: pointer to data buffer
 * @param  startPos: byte position to read
 * @param  size: amount of data elements (u8 or u16) to be read.
 * @retval File function return code
 */
FRESULT SD_ReadFile(char *name, uint8_t *pData, uint32_t startPos, uint32_t size)
{
	static FIL 	fil; //file
	static FRESULT res; //
	static UINT br; //read/write file pointer variables
	static FILINFO fno; //file information variable

	res = f_stat(name, &fno);
	if(res != FR_OK)
	{	//File not existed
		char *buf = malloc(80*sizeof(char));
		sprintf(buf, "Error! File -%s- is not existing to read!\r\n\r\n", name);
		print(buf);
		free(buf);
		return res;
	}
	else
	{	//File is existing
		//Open file to read data
		res = f_open(&fil, name, FA_READ);
		if(res != FR_OK)
		{	//Error
			char *buf = malloc(80*sizeof(char));
			sprintf(buf, "Error %d while opening file -%s-!\r\n\r\n", res, name);
			print(buf);
			free(buf);
			return res;
		}
		else
		{	//Successful
			f_lseek(&fil, startPos);
			if (f_size(&fil) < size)
				size = f_size(&fil);
			else __NOP();
			res = f_read(&fil, pData, size, &br);
			if(res != FR_OK)
			{	//Error
				char *buf = malloc(80*sizeof(char));
				sprintf(buf, "Error %d while reading file -%s-!\r\n\r\n", res, name);
				print(buf);
				free(buf);
			}
			else
			{	//Successful
				print("File content:\r\n");
				print((char*) pData);
				print("\r\n");
			}
			//Close file
			res = f_close(&fil);
			if(res != FR_OK)
			{	//Error
				char *buf = malloc(80*sizeof(char));
				sprintf(buf, "Error %d while closing file -%s-!\r\n\r\n", res, name);
				print(buf);
				free(buf);
			}
		}
	}
	return res;
}

/**
 * @brief  Update new data into the file.
 * @param  name: name of file
 * @param  data: data want to update
 * @retval File function return code
 */
FRESULT SD_UpdateFile(char *name, char *data, uint32_t size)
{
	static FIL 	fil; //file
	static FRESULT res; //
	static UINT bw; //read/write file pointer variables
	static FILINFO fno; //file information variable

	res = f_stat(name, &fno);
	if(res != FR_OK)
	{	//File not existed
		char *buf = malloc(80*sizeof(char));
		sprintf(buf, "Error! File -%s- is not existing to update!\r\n\r\n", name);
		print(buf);
		free(buf);
		return res;
	}
	else
	{	//File is existing
		//Open file to update data
		res = f_open(&fil, name, FA_OPEN_EXISTING | FA_READ | FA_WRITE);
		if(res != FR_OK)
		{	//Error
			char *buf = malloc(80*sizeof(char));
			sprintf(buf, "Error %d while opening file -%s-!\r\n\r\n", res, name);
			print(buf);
			free(buf);
			return res;
		}
		else
		{	//Successful
			//Move pointer to end of file
			res = f_lseek(&fil, f_size(&fil));
			if(res != FR_OK)
			{
				char *buf = malloc(80*sizeof(char));
				sprintf(buf, "Error %d while seeking file -%s-!\r\n\r\n", res, name);
				print(buf);
				free(buf);
			}
			//Write new data
//			if(f_puts(data, &fil) < 0)
			res = f_write(&fil, data, size, &bw);
			if(res != FR_OK)
			{
				char *buf = malloc(80*sizeof(char));
				sprintf(buf, "Error %d while updating new data into file -%s-!\r\n\r\n", res, name);
				print(buf);
				free(buf);
				return res;
			}
			else
			{
				char *buf = malloc(80*sizeof(char));
				sprintf(buf, "File -%s- was updated successfully.\r\n\r\n", name);
				print(buf);
				free(buf);
			}
			//Close file
			res = f_close(&fil);
			if(res != FR_OK)
			{	//Error
				char *buf = malloc(80*sizeof(char));
				sprintf(buf, "Error %d while closing file -%s-!\r\n\r\n", res, name);
				print(buf);
				free(buf);
			}
		}
	}
	return res;
}

/**
 * @brief  Remove the file.
 * @param  name: name of file
 * @retval File function return code
 */
FRESULT SD_DeleteFile(char *name)
{
	static FRESULT res; //
	static FILINFO fno; //file information variable

	res = f_stat(name, &fno);
	if(res != FR_OK)
	{	//File not existed
		char *buf = malloc(80*sizeof(char));
		sprintf(buf, "Error! File -%s- is not existing to delete!\r\n\r\n", name);
		print(buf);
		free(buf);
		return res;
	}
	else
	{	//File is existing
		res = f_unlink(name);
		if(res == FR_OK)
		{
			char *buf = malloc(80*sizeof(char));
			sprintf (buf, "File -%s- was deleted successfully.\r\n\r\n", name);
			print(buf);
			free(buf);
		}
		else
		{
			char *buf = malloc(80*sizeof(char));
			sprintf (buf, "Error %d while deleting file -%s-!\r\n\r\n", res, name);
			print(buf);
			free(buf);
		}
	}
	return res;
}

/**
 * @brief  Rename directory or file.
 * @param  pathOld: Pointer to the object to be renamed
 * @param  pathNew: Pointer to the new name
 * @retval File function return code
 */
FRESULT SD_Rename(const TCHAR* pathOld, const TCHAR* pathNew)
{
	static FRESULT res; //
	static FILINFO fno; //file information variable

	res = f_stat(pathOld, &fno);
	if(res != FR_OK)
	{	//File not existed
		char *buf = malloc(80*sizeof(char));
		sprintf(buf, "Error! File -%s- is not existing to delete!\r\n\r\n", pathOld);
		print(buf);
		free(buf);
		return res;
	}
	else
	{	//File is existing
		res = f_rename(pathOld, pathNew);
		if(res == FR_OK)
		{
			char *buf = malloc(80*sizeof(char));
			sprintf (buf, "File -%s- was renamed successfully. New name is -%s-.\r\n\r\n", pathOld, pathNew);
			print(buf);
			free(buf);
		}
		else
		{
			char *buf = malloc(80*sizeof(char));
			sprintf (buf, "Error %d while renaming file -%s-!\r\n\r\n", res, pathOld);
			print(buf);
			free(buf);
		}
	}
	return res;
}

/**
 * @brief  Rename directory or file.
 * @param  name: name of file
 * @retval size: size of file
 */
uint32_t SD_GetFileSize(char *name)
{
	static FIL 	fil; //file
	static FRESULT res; //
	static FILINFO fno; //file information variable
	uint32_t size = 0;

	res = f_stat(name, &fno);
	if(res != FR_OK)
	{	//File not existed
		char *buf = malloc(80*sizeof(char));
		sprintf(buf, "Error! File -%s- is not existing to read!\r\n\r\n", name);
		print(buf);
		free(buf);
		return res;
	}
	else
	{	//File is existing
		//Open file to read data
		res = f_open(&fil, name, FA_READ);
		if(res != FR_OK)
		{	//Error
			char *buf = malloc(80*sizeof(char));
			sprintf(buf, "Error %d while opening file -%s-!\r\n\r\n", res, name);
			print(buf);
			free(buf);
			return res;
		}
		else
		{	//Successful
			size = fil.fsize;
			char *buf = malloc(80*sizeof(char));
			sprintf(buf, "Size of file -%s- is %lu bytes.\r\n\r\n", name, size);
			print(buf);
			free(buf);
			//Close file
			res = f_close(&fil);
			if(res != FR_OK)
			{	//Error
				char *buf = malloc(80*sizeof(char));
				sprintf(buf, "Error %d while closing file -%s-!\r\n\r\n", res, name);
				print(buf);
				free(buf);
			}
		}
	}
	return size;
}

/**
 * @brief  Creates a directory.
 * @param  name: name of directory
 * @retval File function return code
 */
FRESULT SD_CreateDir(char *name)
{
	static FRESULT res; //

	res = f_mkdir(name);
	if(res == FR_OK)
	{
		char buf[50];
		sprintf(buf, "Directory: %s was created successfully.\r\n\r\n", name);
		print(buf);
	}
	else
	{
		char buf[50];
		sprintf(buf, "Directory: %s wasn't created successfully!\r\n\r\n", name);
		print(buf);
	}
	return res;
}


