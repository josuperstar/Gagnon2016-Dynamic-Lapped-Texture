
#include <vector.h>
#include "Set.h"
//#include "mdebug.h"
using namespace Math;
#include <assert.h>



Set::Set ()
{
}


void Set::FreeBuffers ()
{
	for (int n=0; n < (int) mBuf.size(); n++) {
		free ( mBuf[n].data );
		mBuf[n].data = 0x0;
	}
	mBuf.clear ();
}


int Set::GetSize ()
{
	int sum = 0;
	for (int n=0; n < (int) mBuf.size(); n++)
		sum += mBuf[n].size;	
	return sum;
}


int Set::CopyBuffer ( uchar bdest, uchar bsrc, Set& src )
{
	if ( bsrc >= src.GetNumBuf() ) return -1;

	if ( bdest >= GetNumBuf() ) {
		for (int n=0; n <= bdest - GetNumBuf(); n++ )
			AddBuffer ( 0, 0, 0 );
	}
	if ( mBuf[bdest].data != 0x0 ) {
		free ( mBuf[bdest].data );
		mBuf[bdest].data = 0x0;
	}
		
	GeomBuf* buf = src.GetBuffer( bsrc );
	mBuf[bdest].dtype = buf->dtype;
	mBuf[bdest].max = buf->max;
	mBuf[bdest].num = buf->num;
	mBuf[bdest].stride = buf->stride;
	mBuf[bdest].size = buf->size;
	mBuf[bdest].data = (char*) malloc (  mBuf[bdest].max * mBuf[bdest].stride );
	memcpy ( mBuf[bdest].data, buf->data, mBuf[bdest].num * mBuf[bdest].stride );

	return bdest;
}


void Set::CopyBuffers ( Set& src )
{
	FreeBuffers ();
	for (int n = 0; n < src.GetNumBuf(); n++)
		CopyBuffer ( n, n, src );
}


int Set::AddBuffer ( uchar typ, ushort stride, int max )
{
	GeomBuf buf;
	buf.dtype = typ;
	buf.stride = stride;
	buf.max = max;
	buf.num = 0;
	buf.size = 0;
	buf.data = (char*) malloc ( buf.max * buf.stride );
	mBuf.push_back ( buf );
	return (int) mBuf.size()-1;
}


void Set::ResetBuffer ( uchar b, int n )
{
	mBuf[b].max = n;		

	if ( mBuf[b].data != 0x0 ) free ( mBuf[b].data );

	char* new_data = (char*) malloc ( mBuf[b].max * mBuf[b].stride );
	
	mBuf[b].data = new_data;
	
	mBuf[b].num = 0;
	mBuf[b].size = mBuf[b].num*mBuf[b].stride;
}


char* Set::AddElem ( uchar b, href& ndx )
{
	if ( mBuf[b].num >= mBuf[b].max ) {
		if ( long(mBuf[b].max) * 2 > ELEM_MAX ) {
			//error.PrintF ( "geom", "Maximum number of elements reached.\n" );
			//error.Exit ();
		}
		mBuf[b].max *= 2;		
		char* new_data = (char*) malloc ( mBuf[b].max * mBuf[b].stride );
		memcpy ( new_data, mBuf[b].data, mBuf[b].num*mBuf[b].stride );
		free ( mBuf[b].data );
		mBuf[b].data = new_data;
	}
	mBuf[b].num++;
	mBuf[b].size += mBuf[b].stride; 
	ndx = mBuf[b].num-1;
	return mBuf[b].data + ndx*mBuf[b].stride;
}


char* Set::RandomElem ( uchar b, href& ndx )
{
	ndx = mBuf[b].num * rand() / RAND_MAX;
	return mBuf[b].data + ndx*mBuf[b].stride;
}


int Set::AddElem ( uchar b, char* data )
{
	if ( mBuf[b].num >= mBuf[b].max ) {
		mBuf[b].max *= 2;
		char* new_data = (char*) malloc ( mBuf[b].max * mBuf[b].stride );
		memcpy ( new_data, mBuf[b].data, mBuf[b].num*mBuf[b].stride );
		free ( mBuf[b].data );
		mBuf[b].data = new_data;
	}
	memcpy ( mBuf[b].data + mBuf[b].num*mBuf[b].stride, data, mBuf[b].stride );
	mBuf[b].num++;
	mBuf[b].size += mBuf[b].stride; 
	return mBuf[b].num-1;
}


bool Set::DelElem ( uchar b, int n )
{
	if ( n >= 0 && n < mBuf[b].num ) {
		memcpy ( mBuf[b].data + n*mBuf[b].stride, mBuf[b].data + (mBuf[b].num-1)*mBuf[b].stride, mBuf[b].stride );
		mBuf[b].num--;
		mBuf[b].size -= mBuf[b].stride; 
		return true;
	}
	return false;
}
