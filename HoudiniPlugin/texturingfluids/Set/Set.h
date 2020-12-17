#ifndef DEF_SET
	#define DEF_SET

	#include <vector>

	#define	ELEM_MAX			2147483640	// largest number of elements in a buffer (range of hval)
	
	#define BUF_UNDEF			255

	#define FPOS				2			// free position offsets
	typedef unsigned char		uchar;
	typedef unsigned short		ushort;
	typedef signed int			hval;		// values in heap	
	typedef hval				href;		// values are typically references 
	
	class GeomBuf {
	public:
		GeomBuf()	{ dtype = 0; num = 0; max = 0; stride = 0; data = 0x0; }		
		uchar		dtype;
		hval		num;
		hval		max;
		long		size;
		ushort		stride;		
		char*		data;
	};

	
	class Set {
	public:
		Set ();
		virtual ~Set() {}
	
	//	virtual objType GetType ()			{ return 'geom'; }
	
		// Basic geometry setup	
		void FreeBuffers ();
		int CopyBuffer ( uchar bdest, uchar bsrc, Set& src );
		void CopyBuffers ( Set& src );
		void ResetBuffer ( uchar b, int n );
		int AddBuffer ( uchar typ, ushort stride, int max );

		int NumElem ( uchar b )				{ if ( b==BUF_UNDEF) return 0; else return mBuf[b].num; }
		int MaxElem ( uchar b )				{ if ( b==BUF_UNDEF) return 0; else return mBuf[b].max; } 		
		int GetStride ( uchar b )			{ return mBuf[b].stride; }
		char* GetElem ( uchar b, int n )	{ return mBuf[b].data + n*mBuf[b].stride; }
		char* RandomElem ( uchar b, href& ndx );
		char* AddElem ( uchar b, href& pos );
		int AddElem ( uchar b, char* data );		
		bool DelElem ( uchar b, int n );
		char* GetStart ( uchar b )			{ return mBuf[b].data; }
		char* GetEnd ( uchar b )			{ return mBuf[b].data + mBuf[b].num*mBuf[b].stride; }
		GeomBuf* GetBuffer ( uchar b )		{ return &mBuf[b]; }

		//int NumElem ( T b )				{ if ( b==BUF_UNDEF) return 0; else return mBuf[b].num; }
		//int MaxElem ( T b )				{ if ( b==BUF_UNDEF) return 0; else return mBuf[b].max; } 		
		////int GetStride ( T b )			{ return mBuf[b].stride; }
		//char* GetElem ( T b, int n )	{ return mBuf[n]; }
		////char* RandomElem ( uchar b, href& ndx );
		//char* AddElem ( T b, href& pos );
		//int AddElem ( T e)	{mBuf->push_back(e);}		
		//bool DelElem ( T e, int n )	{mBuf->erase(n);
		//char* GetStart ( T b )			{ return mBuf.start; }
		//char* GetEnd ( uchar b )			{ return mBuf.end; }
		////GeomBuf* GetBuffer ( uchar b )		{ return &mBuf[b]; }

		int GetNumBuf ()					{ return (int) mBuf.size(); }
		int GetSize ();

	protected:
		std::vector< GeomBuf >		mBuf;
		//std::vector< T >		data;

	};

#endif
