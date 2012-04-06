/**
 * Controls SQL interactions for the Disaster Identification system.
 *
 * @author Ian Lonergan
 */

#include <my_global.h>
#include <mysql.h>
#include <stdlib.h>
#include <string.h>
#include "disasterID_sql.h"

#define __LOGIN_SQL "SELECT name FROM tblUsers WHERE name=?"
#define __NAME_SIZE 15
#define __CREATE_RECORD_SQL "INSERT INTO tblBodies (first_name, last_name, location) VALUES (?,?,?)"
#define __FIRST_NAME_SIZE 15
#define __LAST_NAME_SIZE 20
#define __LOCATION_SIZE 36
#define __QUERY_RECORD_BY_NAME_SQL "SELECT * FROM tblBodies WHERE first_name = ? AND last_name = ?"
#define __ID_SIZE 9
#define __QUERY_RECORD_BY_LOCATION_SQL "SELECT * FROM tblBodies WHERE location = ?"
#define __UPDATE_RECORD_NAME_SQL "UPDATE tblBodies SET first_name=?, last_name=? WHERE id_number=?"
#define __ADD_PICTURE_SQL "INSERT INTO tblPictures (first_name, last_name, data) VALUES (?,?,?)"
#define __QUERY_PICTURE_SQL "SELECT data FROM tblPictures WHERE id_number = ? ORDER BY id_number LIMIT 1"
#define __CONNECT_PICTURE_TO_RECORD_SQL "INSERT INTO tblPictureIdentified (picture_id, body_id) VALUES (?,?)"
#define __MAX_IMAGE_SIZE 1024*1000

//Server Information
MYSQL *conn;
char *server = "mysql.wpi.edu";
char *user = "team5user";
char *password = "CXHqJK";
char *database = "disasterid5";

/**
 * Logs into the server and sets up the connection variable.
 *
 * @author Ian Lonergan
 * @param name login name
 */
int login (char *name)
{
	MYSQL_STMT *stmt;
	MYSQL_BIND bind[1];
	unsigned long strLen = strlen(name);
	//MYSQL_RES *result;
	
	//Setup connection and return error if something goes wrong
	conn = mysql_init(conn);
	if (!mysql_real_connect(conn, server, user, password, database, 0, NULL, 0))
	{
		mysql_close(conn);
		return 1;
	}
	
	//Create prepared statement object and return error if something goes wrong
	stmt = mysql_stmt_init(conn);
	if (!stmt)
	{
		mysql_close(conn);
		return 1;
	}
	if (mysql_stmt_prepare(stmt, __LOGIN_SQL, strlen(__LOGIN_SQL)))
	{
		mysql_close(conn);
		return 1;
	}
	
	//Set up statement parameters
	memset(bind, 0, sizeof(bind));
	bind[0].buffer_type = MYSQL_TYPE_STRING;
	bind[0].buffer = (char *)name;
	bind[0].buffer_length = __NAME_SIZE;
	bind[0].is_null = 0;
	bind[0].length = &strLen;
	
	//bind statement parameters to statement
	if (mysql_stmt_bind_param(stmt, bind))
	{
		mysql_stmt_close(stmt);
		mysql_close(conn);
		return 1;
	}
	
	//perform query
	if (mysql_stmt_execute(stmt))
	{
		mysql_stmt_close(stmt);
		mysql_close(conn);
		return 1;
	}

	mysql_stmt_store_result(stmt);
	int rowCount = mysql_stmt_affected_rows(stmt);
	
	if (rowCount > 0)
	{
		return 0;
	}
	else
	{
		mysql_stmt_close(stmt);
		mysql_close(conn);
		return 2;
	}
}

/**
 * Creates a record with given information.
 *
 * @author Ian Lonergan
 * @param firstName First name of the body
 * @param lastName Last name of the body
 * @param location Location the body was found
 * @param response pre-allocated char array of size[10] to hold new record ID
 */
int createRecord(char *firstName, char *lastName, char *location, char *response)
{
	MYSQL_STMT *stmt;
	MYSQL_BIND bind[3];
	
	unsigned long strLen[3];
	my_bool isNull[3];
	strLen[0] = firstName == NULL ? 0 : strlen(firstName);
	strLen[1] = lastName == NULL ? 0 : strlen(lastName);
	strLen[2] = location == NULL ? 0 : strlen(location);
	isNull[0] = firstName == NULL ? 1 : 0;
	isNull[1] = lastName == NULL ? 1 : 0;
	isNull[2] = location == 0;
	
	//Create prepared statement object and return error if something goes wrong
	stmt = mysql_stmt_init(conn);
	if (!stmt)
	{
		return 1;
	}
	if (mysql_stmt_prepare(stmt, __CREATE_RECORD_SQL, strlen(__CREATE_RECORD_SQL)))
	{
		return 1;
	}
	
	//Set up statement parameters
	memset(bind, 0, sizeof(bind));
	
	bind[0].buffer_type = MYSQL_TYPE_STRING;
	bind[0].buffer = (char *)firstName;
	bind[0].buffer_length = __FIRST_NAME_SIZE;
	bind[0].is_null = &isNull[0];
	bind[0].length = &strLen[0];
	
	bind[1].buffer_type = MYSQL_TYPE_STRING;
	bind[1].buffer = (char *)lastName;
	bind[1].buffer_length = __LAST_NAME_SIZE;
	bind[1].is_null = &isNull[1];
	bind[1].length = &strLen[1];
	
	bind[2].buffer_type = MYSQL_TYPE_STRING;
	bind[2].buffer = (char *)location;
	bind[2].buffer_length = __LOCATION_SIZE;
	bind[2].is_null = &isNull[2];
	bind[2].length = &strLen[2];
	
	//bind statement parameters to statement
	if (mysql_stmt_bind_param(stmt, bind))
	{
		mysql_stmt_close(stmt);
		return 1;
	}
		
	//perform query
	if (mysql_stmt_execute(stmt))
	{
		mysql_stmt_close(stmt);
		return 1;
	}
	
	int resultID = mysql_insert_id(conn);
	sprintf(response,"%09i", resultID);
	
	return 0;
}

/**
 * Queries for records with given first and last name.
 *
 * @author Ian Lonergan
 * @param firstName First name of the body
 * @param lastName Last name of the body
 * @param response List of body entries that match the given first and last name
 */
int queryRecordByName(char *firstName, char *lastName, bodyEntry **response)
{
	MYSQL_STMT *stmt;
	MYSQL_BIND bind[2];
	MYSQL_BIND resultBind[4];
	unsigned long strLen[2];
	strLen[0] = strlen(firstName);
	strLen[1] = strlen(lastName);

	//Create prepared statement object and return error if something goes wrong
	stmt = mysql_stmt_init(conn);
	if (!stmt)
	{
		return 1;
	}
	if (mysql_stmt_prepare(stmt, __QUERY_RECORD_BY_NAME_SQL, strlen(__QUERY_RECORD_BY_NAME_SQL)))
	{
		return 1;
	}

	//Set up statement parameters
	memset(bind, 0, sizeof(bind));

	bind[0].buffer_type = MYSQL_TYPE_STRING;
	bind[0].buffer = (char *)firstName;
	bind[0].buffer_length = __FIRST_NAME_SIZE;
	bind[0].is_null = 0;
	bind[0].length = &strLen[0];

	bind[1].buffer_type = MYSQL_TYPE_STRING;
	bind[1].buffer = (char *)lastName;
	bind[1].buffer_length = __LAST_NAME_SIZE;
	bind[1].is_null = 0;
	bind[1].length = &strLen[1];
	
	//bind statement parameters to statement
	if (mysql_stmt_bind_param(stmt, bind))
	{
		mysql_stmt_close(stmt);
		return 1;
	}
	
	//perform query
	if (mysql_stmt_execute(stmt))
	{
		mysql_stmt_close(stmt);
		return 1;
	}	

	int idNumberResponse;
	char *firstNameResponse = malloc((__FIRST_NAME_SIZE + 1)*sizeof(firstNameResponse));
	char *lastNameResponse = malloc((__LAST_NAME_SIZE + 1)*sizeof(lastNameResponse));
	char *locationResponse = malloc((__LOCATION_SIZE + 1)*sizeof(locationResponse));
	unsigned long length[4];
	my_bool error[4];
	my_bool isNull[4];
	
	//bind return values
	memset(resultBind, 0, sizeof(resultBind));
	
	resultBind[0].buffer_type = MYSQL_TYPE_LONG;
	resultBind[0].buffer = (char *)&idNumberResponse;
	resultBind[0].buffer_length = __ID_SIZE;
	resultBind[0].is_null = &isNull[0];
	resultBind[0].length = &length[0];
	resultBind[0].error = &error[0];

	resultBind[1].buffer_type = MYSQL_TYPE_STRING;
	resultBind[1].buffer = firstNameResponse;
	resultBind[1].buffer_length = __FIRST_NAME_SIZE;
	resultBind[1].is_null = &isNull[1];
	resultBind[1].length = &length[1];
	resultBind[1].error = &error[1];
	
	resultBind[2].buffer_type = MYSQL_TYPE_STRING;
	resultBind[2].buffer = lastNameResponse;
	resultBind[2].buffer_length = __LAST_NAME_SIZE;
	resultBind[2].is_null = &isNull[2];
	resultBind[2].length = &length[2];
	resultBind[2].error = &error[2];

	resultBind[3].buffer_type = MYSQL_TYPE_STRING;
	resultBind[3].buffer = locationResponse	;
	resultBind[3].buffer_length = __LOCATION_SIZE;
	resultBind[3].is_null = &isNull[3];
	resultBind[3].length = &length[3];
	resultBind[3].error = &error[3];
	
	if (mysql_stmt_bind_result(stmt, resultBind) || mysql_stmt_store_result(stmt))
	{
		return 1;
	}
	
	int numRows = mysql_stmt_num_rows(stmt);
	bodyEntry responseTest[numRows];
	
	int count = 0;
	while (!mysql_stmt_fetch(stmt))
	{
		responseTest[count].id = idNumberResponse;
		strncpy(responseTest[count].firstName,firstNameResponse,__FIRST_NAME_SIZE);
		strncpy(responseTest[count].lastName,lastNameResponse,__LAST_NAME_SIZE);
		strncpy(responseTest[count].location,locationResponse,__LOCATION_SIZE);
		count++;
	}
	
	*response = responseTest;
	
	return 0;
}

/**
 * Queries for records with a given location
 *
 * @author Ian Lonergan
 * @param location Location the body was found at
 * @param response List of body entries with a matching location
 */
int queryRecordByLocation(char *location, bodyEntry **response)
{
	MYSQL_STMT *stmt;
	MYSQL_BIND bind[1];
	MYSQL_BIND resultBind[4];
	unsigned long strLen[1];
	strLen[0] = strlen(location);

	//Create prepared statement object and return error if something goes wrong
	stmt = mysql_stmt_init(conn);
	if (!stmt)
	{
		return 1;
	}
	if (mysql_stmt_prepare(stmt, __QUERY_RECORD_BY_LOCATION_SQL, strlen(__QUERY_RECORD_BY_LOCATION_SQL)))
	{
		return 1;
	}

	//Set up statement parameters
	memset(bind, 0, sizeof(bind));

	bind[0].buffer_type = MYSQL_TYPE_STRING;
	bind[0].buffer = (char *)location;
	bind[0].buffer_length = __LOCATION_SIZE;
	bind[0].is_null = 0;
	bind[0].length = &strLen[0];

	
	//bind statement parameters to statement
	if (mysql_stmt_bind_param(stmt, bind))
	{
		mysql_stmt_close(stmt);
		return 1;
	}
	
	//perform query
	if (mysql_stmt_execute(stmt))
	{
		mysql_stmt_close(stmt);
		return 1;
	}	

	int idNumberResponse;
	char *firstNameResponse = malloc((__FIRST_NAME_SIZE + 1)*sizeof(firstNameResponse));
	char *lastNameResponse = malloc((__LAST_NAME_SIZE + 1)*sizeof(lastNameResponse));
	char *locationResponse = malloc((__LOCATION_SIZE + 1)*sizeof(locationResponse));
	unsigned long length[4];
	my_bool error[4];
	my_bool isNull[4];
	
	//bind return values
	memset(resultBind, 0, sizeof(resultBind));
	
	resultBind[0].buffer_type = MYSQL_TYPE_LONG;
	resultBind[0].buffer = (char *)&idNumberResponse;
	resultBind[0].buffer_length = __ID_SIZE;
	resultBind[0].is_null = &isNull[0];
	resultBind[0].length = &length[0];
	resultBind[0].error = &error[0];

	resultBind[1].buffer_type = MYSQL_TYPE_STRING;
	resultBind[1].buffer = firstNameResponse;
	resultBind[1].buffer_length = __FIRST_NAME_SIZE;
	resultBind[1].is_null = &isNull[1];
	resultBind[1].length = &length[1];
	resultBind[1].error = &error[1];
	
	resultBind[2].buffer_type = MYSQL_TYPE_STRING;
	resultBind[2].buffer = lastNameResponse;
	resultBind[2].buffer_length = __LAST_NAME_SIZE;
	resultBind[2].is_null = &isNull[2];
	resultBind[2].length = &length[2];
	resultBind[2].error = &error[2];

	resultBind[3].buffer_type = MYSQL_TYPE_STRING;
	resultBind[3].buffer = locationResponse	;
	resultBind[3].buffer_length = __LOCATION_SIZE;
	resultBind[3].is_null = &isNull[3];
	resultBind[3].length = &length[3];
	resultBind[3].error = &error[3];
	
	if (mysql_stmt_bind_result(stmt, resultBind) || mysql_stmt_store_result(stmt))
	{
		return 1;
	}
	
	int numRows = mysql_stmt_num_rows(stmt);
	bodyEntry responseTest[numRows];
	
	int count = 0;
	while (!mysql_stmt_fetch(stmt))
	{
		responseTest[count].id = idNumberResponse;
		strncpy(responseTest[count].firstName,firstNameResponse,__FIRST_NAME_SIZE);
		strncpy(responseTest[count].lastName,lastNameResponse,__LAST_NAME_SIZE);
		strncpy(responseTest[count].location,locationResponse,__LOCATION_SIZE);
		count++;
	}
	
	*response = responseTest;
	
	return 0;
}

/**
 * Updates the record with the given recordID to have a new first and last name.
 *
 * @author Ian Lonergan
 * @param recordID ID of the record to change
 * @param firstName First name of the body
 * @param lastName Last name of the body
 */
int updateRecordName(int recordID, char *firstName, char *lastName)
{
	MYSQL_STMT *stmt;
	MYSQL_BIND bind[3];
	unsigned long strLen[3];
	my_bool isNull[3];
	strLen[0] = firstName == NULL ? 0 : strlen(firstName);
	strLen[1] = lastName == NULL ? 0 : strlen(lastName);
	strLen[2] = __ID_SIZE;
	isNull[0] = firstName == NULL ? 1 : 0;
	isNull[1] = lastName == NULL ? 1 : 0;
	isNull[2] = 0;
	
	//Create prepared statement object and return error if something goes wrong
	stmt = mysql_stmt_init(conn);
	if (!stmt)
	{
		return 1;
	}
	if (mysql_stmt_prepare(stmt, __UPDATE_RECORD_NAME_SQL, strlen(__UPDATE_RECORD_NAME_SQL)))
	{
		return 1;
	}
	
	//Set up statement parameters
	memset(bind, 0, sizeof(bind));
	
	bind[0].buffer_type = MYSQL_TYPE_STRING;
	bind[0].buffer = (char *)firstName;
	bind[0].buffer_length = __FIRST_NAME_SIZE;
	bind[0].is_null = &isNull[0];
	bind[0].length = &strLen[0];
	
	bind[1].buffer_type = MYSQL_TYPE_STRING;
	bind[1].buffer = (char *)lastName;
	bind[1].buffer_length = __LAST_NAME_SIZE;
	bind[1].is_null = &isNull[1];
	bind[1].length = &strLen[1];
	
	bind[2].buffer_type = MYSQL_TYPE_LONG;
	bind[2].buffer = (char *)&recordID;
	bind[2].buffer_length = __ID_SIZE;
	bind[2].is_null = &isNull[2];
	bind[2].length = &strLen[2];
	
	//bind statement parameters to statement
	if (mysql_stmt_bind_param(stmt, bind))
	{
		mysql_stmt_close(stmt);
		return 1;
	}
		
	//perform query
	if (mysql_stmt_execute(stmt))
	{
		mysql_stmt_close(stmt);
		return 1;
	}
		
	return 0;
}

/**
 * Adds a picture with a given first and last name.
 *
 * @author Ian Lonergan
 * @param firstName First name of the person pictured
 * @param lastName Last name of the person pictured
 * @param buffer Character array holding the raw data for the image
 * @param response pre-allocated char array of size[10] to hold new picture ID
 */
int addPicture(char *firstName, char *lastName, char *buffer, char *response)
{
	MYSQL_STMT *stmt;
	MYSQL_BIND bind[3];
	unsigned long strLen[3];
	my_bool isNull[3];	
	strLen[0] = firstName == NULL ? 0 : strlen(firstName);
	strLen[1] = lastName == NULL ? 0 : strlen(lastName);
	strLen[2] = buffer == NULL ? 0 : strlen(buffer);
	isNull[0] = 0;
	isNull[1] = 0;
	isNull[2] = 0;
	
	//Create prepared statement object and return error if something goes wrong
	stmt = mysql_stmt_init(conn);
	if (!stmt)
	{
		return 1;
	}
	if (mysql_stmt_prepare(stmt, __ADD_PICTURE_SQL, strlen(__ADD_PICTURE_SQL)))
	{
		return 1;
	}
	
	//Set up statement parameters
	memset(bind, 0, sizeof(bind));
	
	bind[0].buffer_type = MYSQL_TYPE_STRING;
	bind[0].buffer = (char *)firstName;
	bind[0].buffer_length = __FIRST_NAME_SIZE;
	bind[0].is_null = &isNull[0];
	bind[0].length = &strLen[0];
	
	bind[1].buffer_type = MYSQL_TYPE_STRING;
	bind[1].buffer = (char *)lastName;
	bind[1].buffer_length = __LAST_NAME_SIZE;
	bind[1].is_null = &isNull[1];
	bind[1].length = &strLen[1];
	
	bind[2].buffer_type = MYSQL_TYPE_BLOB;
	bind[2].buffer = (char *)buffer;
	bind[2].buffer_length = __MAX_IMAGE_SIZE;
	bind[2].is_null = &isNull[2];
	bind[2].length = &strLen[2];
	
	//bind statement parameters to statement
	if (mysql_stmt_bind_param(stmt, bind))
	{
		mysql_stmt_close(stmt);
		return 1;
	}
		
	//perform query
	if (mysql_stmt_execute(stmt))
	{
		mysql_stmt_close(stmt);
		return 1;
	}
	
	int resultID = mysql_insert_id(conn);
	sprintf(response,"%09i", resultID);
	
	return 0;
}

/**
 * Returns the picture with the given pictureID.
 *
 * @author Ian Lonergan
 * @param pictureID ID of the picture to return
 * @param buffer Pre-allocated 1MB character array to hold the picture data
 */
int queryPicture(int pictureID, char *buffer)
{
	MYSQL_STMT *stmt;
	MYSQL_BIND bind[1];
	unsigned long strLen = __ID_SIZE;
	//MYSQL_RES *result;
	
	//Create prepared statement object and return error if something goes wrong
	stmt = mysql_stmt_init(conn);
	if (!stmt)
	{
		mysql_close(conn);
		return 1;
	}
	if (mysql_stmt_prepare(stmt, __QUERY_PICTURE_SQL, strlen(__QUERY_PICTURE_SQL)))
	{
		mysql_close(conn);
		return 1;
	}
	
	//Set up statement parameters
	memset(bind, 0, sizeof(bind));
	bind[0].buffer_type = MYSQL_TYPE_LONG;
	bind[0].buffer = (char *)&pictureID;
	bind[0].buffer_length = __NAME_SIZE;
	bind[0].is_null = 0;
	bind[0].length = &strLen;
	
	//bind statement parameters to statement
	if (mysql_stmt_bind_param(stmt, bind))
	{
		mysql_stmt_close(stmt);
		mysql_close(conn);
		return 1;
	}
	
	//perform query
	if (mysql_stmt_execute(stmt))
	{
		mysql_stmt_close(stmt);
		mysql_close(conn);
		return 1;
	}

	MYSQL_BIND resultBind[4];
	char *dataResponse = malloc(2 * __MAX_IMAGE_SIZE + 1);
	unsigned long length[1];
	my_bool error[1];
	my_bool isNull[1];
	
	//bind return values
	memset(resultBind, 0, sizeof(resultBind));
	
	resultBind[0].buffer_type = MYSQL_TYPE_BLOB;
	resultBind[0].buffer = (char *)dataResponse;
	resultBind[0].buffer_length = 2 * __MAX_IMAGE_SIZE + 1;
	resultBind[0].is_null = &isNull[0];
	resultBind[0].length = &length[0];
	resultBind[0].error = &error[0];
	
	if (mysql_stmt_bind_result(stmt, resultBind) || mysql_stmt_store_result(stmt))
	{
		return 1;
	}

	mysql_stmt_fetch(stmt);
	strncpy(buffer, dataResponse, strlen(dataResponse));

	return 0;
}

/**
 * Creates a new record connecting a picture to a body.
 *
 * @author Ian Lonergan
 * @param pictureID ID of the picture
 * @param bodyID ID of the body
 */
int connectPictureToRecord(int pictureID, int bodyID)
{
	MYSQL_STMT *stmt;
	MYSQL_BIND bind[2];
	unsigned long strLen[2];
	my_bool isNull[2];	
	strLen[0] = __ID_SIZE;
	strLen[1] = __ID_SIZE;
	isNull[0] = 0;
	isNull[1] = 0;
	
	//Create prepared statement object and return error if something goes wrong
	stmt = mysql_stmt_init(conn);
	if (!stmt)
	{
		return 1;
	}
	if (mysql_stmt_prepare(stmt, __CONNECT_PICTURE_TO_RECORD_SQL, strlen(__CONNECT_PICTURE_TO_RECORD_SQL)))
	{
		return 1;
	}
	
	//Set up statement parameters
	memset(bind, 0, sizeof(bind));
	
	bind[0].buffer_type = MYSQL_TYPE_LONG;
	bind[0].buffer = (char *)&pictureID;
	bind[0].buffer_length = __ID_SIZE;
	bind[0].is_null = &isNull[0];
	bind[0].length = &strLen[0];
	
	bind[1].buffer_type = MYSQL_TYPE_LONG;
	bind[1].buffer = (char *)&bodyID;
	bind[1].buffer_length = __ID_SIZE;
	bind[1].is_null = &isNull[1];
	bind[1].length = &strLen[1];
	
	//bind statement parameters to statement
	if (mysql_stmt_bind_param(stmt, bind))
	{
		mysql_stmt_close(stmt);
		return 1;
	}
		
	//perform query
	if (mysql_stmt_execute(stmt))
	{
		mysql_stmt_close(stmt);
		return 1;
	}
	
	return 0;
}

int logout()
{
    mysql_close(conn);
    return 0;
}

/*
int main(int argc, char **argv)
{

	char *response = malloc(10 * sizeof(char));
	bodyEntry *bodyEntryList1,*bodyEntryList2;
	
	int retVal = login("Ian");
	printf("Login retVal: %d\n",retVal);
	
	retVal = createRecord(NULL, NULL, "Location", response);
	printf("Create retVal: %d\nCreate Response: %s\n",retVal,response);
	
	retVal = queryRecordByName("First","Last", &bodyEntryList1);
	printf("query1 retVal: %d\n",retVal);
	
	retVal = queryRecordByLocation("Location", &bodyEntryList2);
	printf("query2 retVal: %d\n",retVal);
	
	retVal = updateRecordName(15, "FirstUp", NULL);
	printf("update1 retVal: %d\n",retVal);
	
	retVal = updateRecordName(16, NULL, "LastUp");
	printf("update2 retVal: %d\n",retVal);
	
	retVal = updateRecordName(14, "FirstUp", "LastUp");
	printf("update3 retVal: %d\n",retVal);

	return 0;
}
*/
