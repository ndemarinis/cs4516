#ifndef __DID_STRUCTS
#define __DID_STRUCTS

typedef struct
{
	char name[16];
} loginStruct;

typedef struct
{
	char firstName[16];
	char lastName[21];
	char location[37];
} createRecordStruct;

typedef struct
{
	char firstName[16];
	char lastName[21];
} queryRecordByNameStruct;

typedef struct
{
	char location[37];
} queryRecordByLocationStruct;

typedef struct
{
	int id;
	char firstName[16];
	char lastName[21];
} updateRecordNameStruct;

typedef struct
{
	char firstName[16];
	char lastName[21];
	char data[1024*1000];
} addPictureStruct;

typedef struct
{
	int id;
} queryPictureStruct;

typedef struct
{
	int picID;
	int bodyID;
} connectPictureToRecordStruct;

typedef struct
{
} logoutStruct;


#endif

