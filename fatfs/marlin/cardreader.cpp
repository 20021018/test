#include "Marlin.h"

#include "cardreader.h"
#include "stepper.h"
#include "temperature.h"
#include "language.h"

#include <stdio.h>


#ifdef SDSUPPORT


CardReader::CardReader()
{
#if _USE_LFN
    fno.lfname = lfn;
    fno.lfsize = sizeof(lfn);
#endif

  sdprinting = false;
  cardOK = false;
  saving = false;
	 
	strcpy(path,"0:");
   
	autostart_stilltocheck=true; //the sd start is delayed, because otherwise the serial cannot answer fast enought to make contact with the hostsoftware.

  //power to SD reader
  #ifdef SDPOWER 
    SET_OUTPUT(SDPOWER); 
    WRITE(SDPOWER,HIGH);
  #endif //SDPOWER
  
  autostart_atmillis=millis()+2000;
}

//列出目录下的所有的gcode文件,path必须指向255自己的数组
FRESULT  CardReader::lsDive(char* path)
{

    FRESULT res;
    int i;
    TCHAR *fn;   /* This function is assuming non-Unicode cfg. */
		uint16_t nf = 0;


    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        i = strlen(path);
        for (;;) {
            res = f_readdir(&dir, &fno);                  /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break; /* Break on error or end of dir */
            if (fno.fname[0] == '.') continue;            /* Ignore dot entry */
					if(fno.fattrib&(AM_SYS|AM_HID)) continue; 		/* Ignore system or hidder file */
#if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;
#else
            fn = fno.fname;
#endif
							
					nf++;
					if(lsAction==LS_SerialPrint) 
					{
							if (fno.fattrib & AM_DIR){                    // It is a directory 
									sprintf(&path[i], "/%s", fn);
									printf("scan file - %s\n\r",path);
									res = lsDive(path);
									if (res != FR_OK) break;
									path[i] = 0;
							} else {                                       // It is a file 
								if(strstr(fn,".gcode")!=NULL){ 								//列出后缀是.gcode的文件名
									printf("scan file - %s/%s\r\n", path, fn);
								}

							}
						}
						if((lsAction==LS_GetFilename)&&(nrFiles==nf))
						{
							//if(*fno.lfname==0)fno.lfname= fno.fname;
							return FR_OK; 
						}
						if(lsAction==LS_Count)
						{
							nrFiles = nf;
						}

        }
    }else{
		printf("scan files error : %d\n\r",res);
	} 
  return res;
}


FRESULT CardReader::OpenDir(DIR *dir,char* path)
{
		FRESULT res;
    res = f_opendir(dir, path);                       /* Open the directory */
    if (res == FR_OK) {
			printf("Open %s dir is ok",path);
		}	
		return res;
}
		
//列出目录中所有的文件,包括文件夹,每次调用返回一个文件名
FRESULT   CardReader::ReadDir(DIR *dir,FILINFO *fno)
{
	FRESULT res;
  res = f_readdir(dir, fno);                   /* Read a directory item */
 // if (res != FR_OK) return res; 
	if(fno->fname[0] == 0) return FR_NO_FILE;  /* Break on error or end of dir */
 // if (fno.fname[0] == '.') continue;             /* Ignore dot entry */
  return res;
}


//(2)列出SD里的所有的gcode文件
void CardReader::ls() 
{
	char path[255]="0:";
  lsAction=LS_SerialPrint;
  lsDive(path);
}

//(3)SD卡的初始化
void CardReader::initsd()
{
	if(cardOK==true){
		return;
	}
// #ifdef SDSLOW
//   if (!card.init(SPI_HALF_SPEED,SDSS))
// #else
//   if (!card.init(SPI_FULL_SPEED,SDSS))
// #endif

	FRESULT res;
	res = f_mount(0,&fs); 	//挂载文件系统 fatfs
	if(res != FR_OK){
		printf("mount filesystem 0 failed : %d\n\r",res);
		return;
	}else{
		printf("mount filesystem 0 success : %d\n\r",res);
	}
	cardOK = true;
	return;
}

//释放当前SD卡
void CardReader::release()
{
  sdprinting = false;
  cardOK = false;
}

void CardReader::startFileprint()
{
  if(cardOK)
  {
    sdprinting = true;
  }
}

void CardReader::pauseSDPrint()
{
  if(sdprinting)
  {
    sdprinting = false;
  }
}


void CardReader::openLogFile(char* name)
{
  logging = true;
	openFile(name, false);
}

uint32_t CardReader::readFile(char *buffer,uint32_t rlength)
{
	uint32_t br = 0;
	FRESULT res = f_read(&file, buffer, rlength, &br); 
	if(res != FR_OK){
		printf("readfile file fail......\n\r");
	}
	return br;
}


//(4)打开一个文件
void CardReader::openFile(char* path,bool read)
{
  if(!cardOK)
    return;
	//sdprinting = true;
  if(read) //读一个文件
  {
		printf("openfile_read file test......\n\r");
		FRESULT res = f_open(&file, path, FA_OPEN_EXISTING | FA_READ);
		if(res != FR_OK){
		printf("openfile_read error : %d\n\r",res);
    }
		else{
			printf("openfile_read success : %d\n\r",res);
		}
  }
  else //write
  { 
		printf("openfile_write file test......\n\r");
		FRESULT res = f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE);
		if(res != FR_OK){
		printf("openfile_write error : %d\n\r",res);
    }
		else{
			printf("openfile_write success : %d\n\r",res);
		}
  }
}



//(5)删除一个文件
void CardReader::removeFile(char* name)
{
	FRESULT res = f_unlink(name);
	if(res!=FR_OK){
		printf("removefile fail : %d\n\r",res);
	}
}

//(6)获取打印信息
void CardReader::getStatus()
{
  if(cardOK){
    printf(MSG_SD_PRINTING_BYTE);
    printf("%ld",file.fptr);
    printf("/");
    printf("%d",file.fsize);
  }
  else{
    printf(MSG_SD_NOT_PRINTING);
  }
}


//(7)把buf写入sd卡的数据里面
void CardReader::write_command(char *buf)
{
  char* begin = buf;
  char* npos = 0;
  char* end = buf + strlen(buf) - 1;

//  file.writeError = false;
  if((npos = strchr(buf, 'N')) != NULL)
  {
    begin = strchr(npos, ' ') + 1;
    end = strchr(npos, '*') - 1;
  }
  end[1] = '\r';
  end[2] = '\n';
  end[3] = '\0';
//  file.write(begin);
//  if (file.writeError)
  {
    printf(MSG_ERR);
    printf(MSG_SD_ERR_WRITE_TO_FILE);
		printf("\r\n");
  }
}

void CardReader::checkautostart(bool force)
{
  if(!force)
  {
    if(!autostart_stilltocheck)
      return;
    if(autostart_atmillis>millis())
      return;
  }
	autostart_stilltocheck = false;
  if(!cardOK)
  {
    initsd();
    if(!cardOK) //fail
      return;
  }
  
// char autoname[30];
// sprintf_P(autoname, PSTR("auto%i.g"), lastnr);
// for(int8_t i=0;i<(int8_t)strlen(autoname);i++)
// 	autoname[i]=tolower(autoname[i]);
// dir_t p;

// root.rewind();

// bool found=false;
// while (root.readDir(p, NULL) > 0) 
// {
//  for(int8_t i=0;i<(int8_t)strlen((char*)p.name);i++)
//  p.name[i]=tolower(p.name[i]);
// 	Serial.print((char*)p.name);
// 	Serial.print(" ");
// 	Serial.println(autoname);
// 	if(p.name[9]!='~') //skip safety copies
// 	if(strncmp((char*)p.name,autoname,5)==0)
// 	{
// 		char cmd[30];

// 		sprintf_P(cmd, PSTR("M23 %s"), autoname);
// 		enquecommand(cmd);
// 		enquecommand_P(PSTR("M24"));
// 		found=true;
// 	}
// }
// if(!found)
//  lastnr=-1;
// else
//  lastnr++;
}

int CardReader::savefile(char *pcmdbuf)
{
	if(card.saving)
	{
		if(strstr(pcmdbuf, PSTR("M29")) == NULL)
		{
			card.write_command(pcmdbuf);
			if(card.logging)
			{
				process_commands();
			}
			else
			{
				printf(MSG_OK);
				printf("\r\n");
				return 0;
			}
		}
		else
		{
			card.closefile();
			printf(MSG_FILE_SAVED);
			return 1;
		}
	}
	return 1;
}

void CardReader::closefile()
{
  f_sync(&file);
  f_close(&file);
  saving = false; 
  logging = false;
}


void CardReader::getfilename(const uint8_t nr,char *path)
{
  lsAction=LS_GetFilename; 
  nrFiles=nr;
  lsDive(path);
}

uint16_t CardReader::getfilecount(char *path)
{
	lsAction = LS_Count;
  lsDive(path);
	printf("%d",nrFiles);
  return nrFiles;
}


//
char CardReader::isroot()
{
	if(strchr(path,'/')==NULL)
	{
		return 1;
	}
	return 0;
}

//
void CardReader::chdir(const char * relpath)
{
	strcpy(path,relpath);
}

//删掉字符串中chart字符中第一个chart字符后面的所有的字符
char strdel(char *str,char chart)
{
	while((*str != chart)&&(*str != '\0')){str++;};
	if(*str == '\0') return 0; //没有找到chart字符
	while(*str != '\0')
	{
		*str = '\0';
		str++;
	}
	return 1;
}


void CardReader::updir()
{
	strdel(path,'/');
}

void CardReader::downdir(char *dirname)
{
	strcat(path,"/");
	strcat(path,dirname);
}



void CardReader::printingHasFinished()
{
    st_synchronize();
    quickStop();
		f_close(&file);
    sdprinting = false;
    if(SD_FINISHED_STEPPERRELEASE)
    {
        finishAndDisableSteppers();
        enquecommand_P(PSTR(SD_FINISHED_RELEASECOMMAND));
    }
    autotempShutdown();
}
#endif //SDSUPPORT
