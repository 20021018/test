#ifndef CARDREADER_H
#define CARDREADER_H

#ifdef SDSUPPORT

#include "../FATFS/src/ff.h"
//#include "SPI_MSD1_Driver.h"

enum LsAction {LS_SerialPrint,LS_Count,LS_GetFilename};

#define LONG_FILENAME_LENGTH 256
#define FILE_VALID_NAME *card.fno.lfname?card.fno.lfname:card.fno.fname
//#define FILE_VALID_NAME  card.fno.lfname

#define FILE_IS_DIR card.fno.fattrib&AM_DIR
class CardReader
{
public:
  
	CardReader();
  
  void initsd();
  void write_command(char *buf);
  //files auto[0-9].g on the sd card are performed in a row
  //this is to delay autostart and hence the initialisaiton of the sd card to some seconds after the normal init, so the device is available quick after a reset

  void checkautostart(bool x); 
  void openFile(char* name,bool read);
	
	uint32_t readFile(char *buffer,uint32_t rlength);
	void writefile();
  void openLogFile(char* name);
  void removeFile(char* name);
  void closefile();
	int savefile(char *pcmdbuf);
  void release();
  void startFileprint();
  void pauseSDPrint();
  void getStatus();
  void printingHasFinished();

  void getfilename(const uint8_t nr,char *path);
  uint16_t getfilecount(char *path);
  

  void ls();
  void chdir(const char * relpath);
  void updir();
  void downdir(char *dirname);
	char isroot();
	
  FRESULT lsDive(char* path);
	
	FRESULT ReadDir(DIR *dir,FILINFO* flo);
	FRESULT OpenDir(DIR *dir,char* path);


  //FORCE_INLINE bool eof() { return sdpos>=filesize ;};
  //FORCE_INLINE int16_t get() {  sdpos = file.curPosition();return (int16_t)file.read();};  //FORCE_INLINE void setIndex(long index) {sdpos = index;file.seekSet(index);};
  //FORCE_INLINE uint8_t percentDone(){if(!isFileOpen()) return 0; if(filesize) return sdpos/((filesize+99)/100); else return 0;};
  //FORCE_INLINE char* getWorkDirName(){workDir.getFilename(filename);return filename;};
  //FORCE_INLINE bool isFileOpen() { return file.isOpen(); }
	// FORCE_INLINE uint8_t percentDone(){if(!isFileOpen()) return 0; if(file_current.fsize) return file_current.fptr/((file_current.fsize+99)/100); else return 0;};
  //FORCE_INLINE char* getWorkDirName(){workDir.getFilename(filename);return filename;};
	
	FORCE_INLINE bool isFileOpen() { if(file.id)return true;else return false; }
	FORCE_INLINE bool err() {return f_error(&file);};
  FORCE_INLINE bool eof() {return f_eof(&file);};
  FORCE_INLINE int16_t get() {return f_get(&file);};
  FORCE_INLINE void setIndex(long index) {f_size(&file) = index;};
	
	FORCE_INLINE unsigned long size() {return f_size(&file);};
		
	FORCE_INLINE unsigned long getIndex() {return f_tell(&file);};
	
	FORCE_INLINE uint8_t percentDone(){if(!isFileOpen()) return 0; if(file.fsize) return file.fptr/((file.fsize+99)/100); else return 0;};
	

	
public:
  bool saving;
  bool sdprinting ;  
  bool cardOK ;
	bool logging;

	FATFS fs; 	 //文件系统 
	DIR dir;		 //文件夹	
	FIL file;  	 //文件
	FILINFO fno; //文件信息

	char path[_MAX_LFN + 1];
	
private:

#if _USE_LFN
    char lfn[_MAX_LFN + 1]; //保存文件长名
#endif

  unsigned long autostart_atmillis;
  bool autostart_stilltocheck; //the sd start is delayed, because otherwise the serial cannot answer fast enought to make contact with the hostsoftware.
  
  LsAction lsAction; //stored for recursion.
  int16_t nrFiles; //counter for the files in the current directory and recycled as position counter for getting the nrFiles'th name in the directory.
  char* diveDirName;
	


	//char path[_MAX_LFN + 1]="0:"; //用来保存SD卡当前文件路径和名称


};
extern CardReader card;
#define IS_SD_PRINTING (card.sdprinting)

#if (SDCARDDETECT > -1)
# ifdef SDCARDDETECTINVERTED 
#  define IS_SD_INSERTED (READ(SDCD_PIN)!=0)
# else
#  define IS_SD_INSERTED (READ(SDCD_PIN)==0)
# endif //SDCARDTETECTINVERTED
#else
//If we don't have a card detect line, aways asume the card is inserted
# define IS_SD_INSERTED true
#endif

#else

#define IS_SD_PRINTING (false)

#endif //SDSUPPORT

#endif
