/*
 * This file is part of the MCBMini firmware.
 * MCBMini is a complete, open-source, flexible and scalable 
 * motor control scheme with board designs, firmware and host 
 * software. 
 * This is the firmware for MCBMini
 * The MCBMini project can be downloaded from:
 * http://code.google.com/p/mcbmini/ 
 *
 * (c) Sigurdur Orn Adalgeirsson (siggi@alum.mit.edu)
 *
 * MCBMini firmware is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License
 * 
 * MCBMini firmware is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with the MCBMini firmware.  If not, see <http://www.gnu.org/licenses/>.
 */
 
 /*
 * circBuffer.h
 *
 *  Created on: Sep 10, 2012
 *      Author: siggi
 */

#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_

#include <util/atomic.h>

typedef struct _circBuffer
{
	unsigned char *databuffer;			// The actual buffer storage location
	unsigned volatile short size;		// The absolute size
	unsigned volatile short index;		// Is the current index
	unsigned volatile short length;		// Is the current index
} circBuffer;

/*
 * Size must be a multiple of four
 */
void circBufferInit(volatile circBuffer* buffer, unsigned char *_databuffer, unsigned char _size){
	buffer->databuffer = _databuffer;
	buffer->size = _size;
	buffer->index = 0;
	buffer->length = 0;
}

void circBufferReset(volatile circBuffer* buffer){
	ATOMIC_BLOCK(ATOMIC_FORCEON){
		buffer->index = 0;
		buffer->length = 0;
	}
}

unsigned short circBufferFree(volatile circBuffer* buffer){
	unsigned short ret;
	ATOMIC_BLOCK(ATOMIC_FORCEON){
		ret = buffer->size-buffer->length;
	}
	return ret;
}

void circBufferPut(volatile circBuffer* buffer, unsigned char value){
	ATOMIC_BLOCK(ATOMIC_FORCEON){
		buffer->databuffer[(buffer->index+buffer->length)%buffer->size] = value;
		if( buffer->length == buffer->size ){
			buffer->index = (buffer->index+1)%buffer->size;
		}
		else{
			buffer->length++;
		}
	}
}

unsigned char circBufferPeekAtIndex(volatile circBuffer* buffer, unsigned short index){
	unsigned char ret;
	ATOMIC_BLOCK(ATOMIC_FORCEON){
		ret = buffer->databuffer[(buffer->index+index)%buffer->size];
	}
	return ret;
}

unsigned char circBufferPeekFirst(volatile circBuffer* buffer){
	return circBufferPeekAtIndex(buffer, 0);
}

unsigned char circBufferGetFirst(volatile circBuffer* buffer){
	unsigned char ret = circBufferPeekAtIndex(buffer, 0);
	ATOMIC_BLOCK(ATOMIC_FORCEON){
		if( buffer->length > 0 ){
			buffer->index = (buffer->index+1)%buffer->size;
			buffer->length--;
		}
	}
	return ret;
}

unsigned char circBufferGetLast(volatile circBuffer* buffer){
	ATOMIC_BLOCK(ATOMIC_FORCEON){
		if( buffer->length > 0 ){
			buffer->length--;
			return circBufferPeekAtIndex(buffer, buffer->length);
		}
	}
	return 0;
}

void circBufferPutLong(volatile circBuffer* buffer, long value){
	ATOMIC_BLOCK(ATOMIC_FORCEON){
		circBufferPut(buffer, (unsigned char)((value>>24)&0xff));
		circBufferPut(buffer, (unsigned char)((value>>16)&0xff));
		circBufferPut(buffer, (unsigned char)((value>>8)&0xff));
		circBufferPut(buffer, (unsigned char)((value)&0xff));
	}
}

long circBufferPeekLongAtIndex(volatile circBuffer* buffer, unsigned short index){
	long value;
	ATOMIC_BLOCK(ATOMIC_FORCEON){
		value = (long)circBufferPeekAtIndex(buffer, index ) << 24;
		value += (long)circBufferPeekAtIndex(buffer, index+1 ) << 16;
		value += (long)circBufferPeekAtIndex(buffer, index+2 ) << 8;
		value += (long)circBufferPeekAtIndex(buffer, index+3 );
	}
	return value;
}

long circBufferPeekFirstLong(volatile circBuffer* buffer){
	long value;
	ATOMIC_BLOCK(ATOMIC_FORCEON){
		value = circBufferPeekLongAtIndex(buffer, 0);
	}
	return value;
}

long circBufferGetFirstLong(volatile circBuffer* buffer){
	long val;
	ATOMIC_BLOCK(ATOMIC_FORCEON){
		if( buffer->length >= 4 ){
			val = circBufferPeekFirstLong(buffer);
			buffer->index = (buffer->index+4)%buffer->size;
			buffer->length -= 4;
		}
		else{
			val = 0;
		}
	}
	return val;
}

long circBufferPeekLastLong(volatile circBuffer* buffer){
	long value;
	ATOMIC_BLOCK(ATOMIC_FORCEON){
		value = circBufferPeekLongAtIndex(buffer, buffer->length-4);
	}
	return value;
}


#endif /* CIRCULARBUFFER_H_ */
