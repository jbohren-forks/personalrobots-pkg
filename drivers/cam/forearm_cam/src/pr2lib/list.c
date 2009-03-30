/*
 * @file list.c
 *
 */
#include <stdlib.h>
#include "list.h"
#include "host_netutil.h"

/**
 * Initializes a new camera list using macros from list.h
 *
 * @param ipCamList Pointer to an IpCamList list element
 *
 * @return Always returns zero.
 */
int pr2CamListInit( IpCamList *ipCamList ) {
	INIT_LIST_HEAD(&ipCamList->list);
	return 0;
}


/**
* Adds a new camera to camera list ipCamList if that serial number is not
* already in the list.
*
* @param ipCamList 	Pointer to the IpCamList head
* @param newItem	Pointer to an IpCamList structure that describes the new camera
*
* @return Returns CAMLIST_ADD_OK if the camera was added to the list, or CAMLIST_ADD_DUP if it was a duplicate.
*/
int pr2CamListAdd( IpCamList *ipCamList, IpCamList *newItem) {
	IpCamList *listIterator;

	int isInList = 0;

	// Scan through the list, looking for a serial number that matches the one we're trying to add
	list_for_each_entry(listIterator, &(ipCamList->list), list) {
		if( newItem->serial == listIterator->serial) {
			// This camera is already in the list
			isInList = 1;
			break;
		}
	}

	if(isInList == 1) {
		debug("Serial number %d already exists in list, dropping.\n", newItem->serial);
		return CAMLIST_ADD_DUP;
	} else {
		debug("Serial number %d is new, adding to list.\n", newItem->serial);
		list_add_tail( &(newItem->list), &(ipCamList->list) );
		return CAMLIST_ADD_OK;
	}

}

/**
 * Utility function to locate a camera with a particular serial number in a list.
 * @param ipCamList	Pointer to the list head
 * @param serial	Serial number (not including product ID) to look for
 * @return			Returns the index of the camera, or -1 if not found
 */
int pr2CamListFind( IpCamList *ipCamList, uint32_t serial ) {
	int count;

	IpCamList *listIterator;
	count = 0;

	list_for_each_entry(listIterator, &(ipCamList->list), list) {
		if(listIterator->serial == serial) {
			return count;
		}
		count++;
	}

	return -1;
}

/**
 * Utility function to return a specific element number from the camera list.
 *
 * @pre index must be less than the value returned by pr2CamListNumEntries().
 * If it is greater, then the last element in the list will be returned.
 *
 * @param ipCamList 	Pointer to the camera list head
 * @param index		Number of the list element to returna (0..max)
 *
 * @return Returns a pointer to the requested list element
 */
IpCamList *pr2CamListGetEntry( const IpCamList *ipCamList, int index ) {
	IpCamList *listIterator;

	// Iterate through the list until we have counted off 'index' entries
	list_for_each_entry(listIterator, &(ipCamList->list), list) {
		if(index-- == 0) {
			break;
		}
	}

	return listIterator;
}

/**
 * Utility function to remove a specific element number from the camera list.
 *
 * @pre index must be less than the value returned by pr2CamListNumEntries().
 *
 * @param ipCamList 	Pointer to the camera list head
 * @param index		Number of the list element to remove (0..max)
 *
 * @return Returns 0 if successful, -1 if index was invalid
 */
int pr2CamListDelEntry( IpCamList *ipCamList, int index ) {
	int count;

	IpCamList *tmpListItem;
	struct list_head *pos, *q;
	count = 0;

	list_for_each_safe(pos, q,&(ipCamList->list)) {
		if(count++ == index) {
			tmpListItem = list_entry(pos, IpCamList, list);
			list_del(pos);
			free(tmpListItem);
			return 0;
		}
	}
	return -1;
}

/**
 * Utility function to determine the number of entries in an IpCamlist
 *
 * @param ipCamList	Pointer to the list head to count.
 *
 * @return Returns the number of elements in the list
 */
int pr2CamListNumEntries( const IpCamList *ipCamList ) {
	int count;

	IpCamList *listIterator;
	count = 0;

	list_for_each_entry(listIterator, &(ipCamList->list), list) {
		count++;
	}

	return count;
}
