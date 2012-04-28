/**
 * Header file for interacting with the SQL database.
 *
 * @author Ian Lonergan
 */

#ifndef __DISASTERID_SQL
#define __DISASTERID_SQL

/*typedef struct
{
	int id;
	char firstName[16];
	char lastName[21];
	char location[37];
} bodyEntry;*/

typedef struct
{
	int id;
	char firstName[16];
	char lastName[21];
	char location[37];
} bodyEntry;


#define __NAME_SIZE 15
#define __FIRST_NAME_SIZE 15
#define __LAST_NAME_SIZE 20
#define __LOCATION_SIZE 36
#define __ID_SIZE 9
#define __MAX_IMAGE_SIZE (1024*1000)

int login (char *);
int createRecord(char *, char *, char *, char *);
int queryRecordByName(char *, char *, bodyEntry **, int *);
int queryRecordByLocation(char *, bodyEntry **, int *);
int updateRecordName(int, char *, char *);
int addPicture(char *, char *, char *, char *);
int queryPicture(int, char *);
int connectPictureToRecord(int, int);
int logout();

#endif

