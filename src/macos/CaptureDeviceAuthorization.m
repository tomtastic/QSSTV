/**************************************************************************
*   Copyright (C) 2023 by Mario Klebsch                                   *
*   mario@klebsch.de                                                      *
*   http://klebsch.de                                                     *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program; if not, write to the                         *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/

#include "CaptureDeviceAuthorization.h"

#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>

void CaptureDeviceAuthorizationStatus(void(*cb)(int, void*), void *context)
{
	switch([AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeAudio])
	{
		case AVAuthorizationStatusRestricted:
			printf("XXX: %s(): AuthorizationStatusRestricted\n", __FUNCTION__);
			cb(false, context);
			return;
		case AVAuthorizationStatusDenied:
			printf("XXX: %s(): AVAuthorizationStatusDenied\n", __FUNCTION__);
			cb(false, context);
			return;
			
		case AVAuthorizationStatusAuthorized:
			printf("XXX: %s(): AVAuthorizationStatusAuthorized\n", __FUNCTION__);
			cb(true, context);
			return;
			
		case AVAuthorizationStatusNotDetermined:
			printf("XXX: %s(): AVAuthorizationStatusNotDetermined\n", __FUNCTION__);
			[AVCaptureDevice requestAccessForMediaType:AVMediaTypeAudio completionHandler:^(BOOL granted) {
				cb(granted, context);
			}];
	}
}

