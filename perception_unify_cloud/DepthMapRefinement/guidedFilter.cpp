#include "filter.h"

using namespace std;
static const int BORDER_TYPE = cv::BORDER_REPLICATE;
//static const int BORDER_TYPE = cv::BORDER_DEFAULT;

//#define FLOAT_BOX_FILTER_OFF 1
//#define STATICMAT static Mat
#define STATICMAT Mat

/****************************************************************************************\
                                         Box Filter
\****************************************************************************************/

template<typename T, typename ST> struct RowSum : public BaseRowFilter
{
    RowSum( int _ksize, int _anchor )
    {
        ksize = _ksize;
        anchor = _anchor;
    }

    void operator()(const uchar* src, uchar* dst, int width, int cn)
    {
        const T* S = (const T*)src;
        ST* D = (ST*)dst;
        int i = 0, k, ksz_cn = ksize*cn;

        width = (width - 1)*cn;
        for( k = 0; k < cn; k++, S++, D++ )
        {
            ST s = 0;
            for( i = 0; i < ksz_cn; i += cn )
                s += S[i];
            D[0] = s;
            for( i = 0; i < width; i += cn )
            {
                s += S[i + ksz_cn] - S[i];
                D[i+cn] = s;
            }
        }
    }
};

template<typename ST, typename T> struct ColumnSum : public BaseColumnFilter
{
    ColumnSum( int _ksize, int _anchor, double _scale )
    {
        ksize = _ksize;
        anchor = _anchor;
        scale = _scale;
        sumCount = 0;
    }

    void reset() { sumCount = 0; }

    void operator()(const uchar** src, uchar* dst, int dststep, int count, int width)
    {
        int i;
        ST* SUM;
        bool haveScale = scale != 1;
        double _scale = scale;

        if( width != (int)sum.size() )
        {
            sum.resize(width);
            sumCount = 0;
        }

        SUM = &sum[0];
        if( sumCount == 0 )
        {
            for( i = 0; i < width; i++ )
                SUM[i] = 0;
            for( ; sumCount < ksize - 1; sumCount++, src++ )
            {
                const ST* Sp = (const ST*)src[0];
                for( i = 0; i <= width - 2; i += 2 )
                {
                    ST s0 = SUM[i] + Sp[i], s1 = SUM[i+1] + Sp[i+1];
                    SUM[i] = s0; SUM[i+1] = s1;
                }

                for( ; i < width; i++ )
                    SUM[i] += Sp[i];
            }
        }
        else
        {
            CV_Assert( sumCount == ksize-1 );
            src += ksize-1;
        }

        for( ; count--; src++ )
        {
            const ST* Sp = (const ST*)src[0];
            const ST* Sm = (const ST*)src[1-ksize];
            T* D = (T*)dst;
            if( haveScale )
            {
                for( i = 0; i <= width - 2; i += 2 )
                {
                    ST s0 = SUM[i] + Sp[i], s1 = SUM[i+1] + Sp[i+1];
                    D[i] = saturate_cast<T>(s0*_scale);
                    D[i+1] = saturate_cast<T>(s1*_scale);
                    s0 -= Sm[i]; s1 -= Sm[i+1];
                    SUM[i] = s0; SUM[i+1] = s1;
                }

                for( ; i < width; i++ )
                {
                    ST s0 = SUM[i] + Sp[i];
                    D[i] = saturate_cast<T>(s0*_scale);
                    SUM[i] = s0 - Sm[i];
                }
            }
            else
            {
                for( i = 0; i <= width - 2; i += 2 )
                {
                    ST s0 = SUM[i] + Sp[i], s1 = SUM[i+1] + Sp[i+1];
                    D[i] = saturate_cast<T>(s0);
                    D[i+1] = saturate_cast<T>(s1);
                    s0 -= Sm[i]; s1 -= Sm[i+1];
                    SUM[i] = s0; SUM[i+1] = s1;
                }

                for( ; i < width; i++ )
                {
                    ST s0 = SUM[i] + Sp[i];
                    D[i] = saturate_cast<T>(s0);
                    SUM[i] = s0 - Sm[i];
                }
            }
            dst += dststep;
        }
    }

    double scale;
    int sumCount;
    vector<ST> sum;
};



template<> struct ColumnSum<float, float> : public BaseColumnFilter
{
    ColumnSum( int _ksize, int _anchor, double _scale )
    {
        ksize = _ksize;
        anchor = _anchor;
        scale = _scale;
        sumCount = 0;
    }

    void reset() { sumCount = 0; }

    void operator()(const uchar** src, uchar* dst, int dststep, int count, int width)
    {
        int i;
        float* SUM;
        bool haveScale = scale != 1;
        double _scale = scale;
	
		#if CV_SSE2
			bool haveSSE2 = checkHardwareSupport(CV_CPU_SSE2);	
		#endif

        if( width != (int)sum.size() )
        {
            sum.resize(width);
            sumCount = 0;
        }

        SUM = &sum[0];
        if( sumCount == 0 )
        {
			memset((void*)SUM, 0, width*sizeof(float));
            for( ; sumCount < ksize - 1; sumCount++, src++ )
            {
                const float* Sp = (const float*)src[0];
				i = 0;
				#if CV_SSE2
				if(haveSSE2)
				{
					for( ; i < width-4; i+=4 )
					{
						__m128 _sum = _mm_loadu_ps((SUM+i));
						__m128 _sp = _mm_loadu_ps((Sp+i));
						_mm_storeu_ps((SUM+i),_mm_add_ps(_sum, _sp));
					}
				}
				#endif
                for( ; i < width; i++ )
                    SUM[i] += Sp[i];
            }
        }
        else
        {
            CV_Assert( sumCount == ksize-1 );
            src += ksize-1;
        }

		
        for( ; count--; src++ )
        {
            const float* Sp = (const float*)src[0];
            const float* Sm = (const float*)src[1-ksize];
            float* D = (float*)dst;
            if( haveScale )
            {
				i = 0;
				#if CV_SSE2
				if(haveSSE2)
				{
					const __m128 scale4 = _mm_set1_ps((float)_scale);
					for( ; i < width-4; i+=4 )
					{
						__m128 _sm  = _mm_loadu_ps((Sm+i));
						__m128 _s0  = _mm_add_ps(_mm_loadu_ps((SUM+i)),
                                                     _mm_loadu_ps((Sp+i)));
						_mm_storeu_ps(D+i, _mm_mul_ps(scale4,_s0));
						_mm_storeu_ps(SUM+i,_mm_sub_ps(_s0,_sm));
					}
				}
				#endif
                for( ; i < width; i++ )
                {
                    float s0 = SUM[i] + Sp[i];
                    D[i] = saturate_cast<float>(s0*_scale);
                    SUM[i] = s0 - Sm[i];
                }
            }
            else
            {
				i = 0;
				#if CV_SSE2
				if(haveSSE2)
				{
					for( ; i < width-4; i+=4 )
					{
						__m128 _sm  = _mm_loadu_ps((Sm+i));
						__m128 _s0  = _mm_add_ps(_mm_loadu_ps((SUM+i)),
                                                     _mm_loadu_ps((Sp+i)));
						_mm_storeu_ps(D+i,_s0);
						_mm_storeu_ps(SUM+i,_mm_sub_ps(_s0,_sm));
					}
				}
				#endif

                for( ; i < width; i++ )
                {
                    float s0 = SUM[i] + Sp[i];
                    D[i] = saturate_cast<uchar>(s0);
                    SUM[i] = s0 - Sm[i];
                }
            }
            dst += dststep;
        }
    }

    double scale;
    int sumCount;
    vector<float> sum;
};

cv::Ptr<cv::FilterEngine> createBoxFilterFFF(int cn, Size ksize,
                    Point anchor, bool normalize, int borderType )
{
    Ptr<BaseRowFilter> rowFilter = Ptr<BaseRowFilter>(new RowSum<float, float>(ksize.width, anchor.x < 0 ? ksize.width/2 : anchor.x));
	
	Ptr<BaseColumnFilter> columnFilter = Ptr<BaseColumnFilter>(new ColumnSum<float, float>(ksize.height, anchor.y < 0 ? ksize.height/2 : anchor.y, normalize ? 1./(ksize.width*ksize.height) : 1));

    return Ptr<FilterEngine>(new FilterEngine(Ptr<BaseFilter>(0), rowFilter, columnFilter,
           CV_32F, CV_32F, CV_32F, borderType ));
}


void boxFilter2( InputArray _src, OutputArray _dst, int ddepth,
                Size ksize, Point anchor,
                bool normalize, int borderType )
{

    Mat src = _src.getMat();
    int sdepth = src.depth(), cn = src.channels();
    if( ddepth < 0 )
        ddepth = sdepth;
    _dst.create( src.size(), CV_MAKETYPE(ddepth, cn) );
    Mat dst = _dst.getMat();
    if( borderType != BORDER_CONSTANT && normalize )
    {
        if( src.rows == 1 )
            ksize.height = 1;
        if( src.cols == 1 )
            ksize.width = 1;
    }

	//src.copyTo(dst);return;
    Ptr<FilterEngine> f = createBoxFilterFFF( cn, ksize, anchor, normalize, borderType );
    f->apply( src, dst );
}

void boxFilter1x1( InputArray _src, OutputArray _dst, int ddepth,
                Size ksize, Point anchor,
                bool normalize, int borderType )
{

    Mat src = _src.getMat();
    int sdepth = src.depth(), cn = src.channels();
    if( ddepth < 0 )
        ddepth = sdepth;
    _dst.create( src.size(), CV_MAKETYPE(ddepth, cn) );
    Mat dst = _dst.getMat();
    if( borderType != BORDER_CONSTANT && normalize )
    {
        if( src.rows == 1 )
            ksize.height = 1;
        if( src.cols == 1 )
            ksize.width = 1;
    }

	src.copyTo(dst);
}

static void multiplySSE_float(Mat& src1, const float amp, Mat& dest)
{

	float* s1 = src1.ptr<float>(0);
	float* d = dest.ptr<float>(0);
	const int size = src1.size().area()/4;
	const __m128 ms2 = _mm_set_ps1(amp);
	const int nn = src1.size().area()-size*4;
	if(src1.data==dest.data)
	{
		for(int i=size;i--;s1+=4)
		{
			__m128 ms1 = _mm_load_ps(s1);
			ms1 = _mm_mul_ps(ms1,ms2);
			_mm_store_ps(s1,ms1);
		}
	}
	else
	{
		for(int i=size;i--;)
		{
			__m128 ms1 = _mm_load_ps(s1);
			ms1 = _mm_mul_ps(ms1,ms2);
			_mm_store_ps(d,ms1);

			s1+=4,d+=4;
		}
	}
	for(int i=0;i<nn;i++)
	{
		*d++ = *s1++ * amp;
	}
}



static void multiplySSE_float(Mat& src1, Mat& src2, Mat& dest)
{
	float* s1 = src1.ptr<float>(0);
	float* s2 = src2.ptr<float>(0);
	float* d = dest.ptr<float>(0);
	const int size = src1.size().area()/4;
	const int nn = src1.size().area()-size*4;
	for(int i=size;i--;)
	{
		__m128 ms1 = _mm_load_ps(s1);
		__m128 ms2 = _mm_load_ps(s2);
		ms1 = _mm_mul_ps(ms1,ms2);
		_mm_store_ps(d,ms1);
		s1+=4,s2+=4,d+=4;
	}
	for(int i=0;i<nn;i++)
	{
		*d++ = *s1++ * *s2++;
	}
}
static void multiplySSEStream_float(Mat& src1, Mat& src2, Mat& dest)
{
	float* s1 = src1.ptr<float>(0);
	float* s2 = src2.ptr<float>(0);
	float* d = dest.ptr<float>(0);
	const int size = src1.size().area()/4;
	const int nn = src1.size().area()-size*4;
	for(int i=size;i--;)
	{
		__m128 ms1 = _mm_load_ps(s1);
		__m128 ms2 = _mm_load_ps(s2);
		ms1 = _mm_mul_ps(ms1,ms2);
		_mm_stream_ps(d,ms1);

		s1+=4,s2+=4,d+=4;
	}
	for(int i=0;i<nn;i++)
	{
		*d++ = *s1++ * *s2++;
	}
}

static void multiplySSE_float(Mat& src1, Mat& dest)
{
	float* s1 = src1.ptr<float>(0);
	float* d = dest.ptr<float>(0);
	const int size = src1.size().area()/4;
	const int nn = src1.size().area()-size*4;
	for(int i=size;i--;)
	{
		__m128 ms1 = _mm_load_ps(s1);
		ms1 = _mm_mul_ps(ms1,ms1);
		_mm_store_ps(d,ms1);

		s1+=4,d+=4;
	}
	for(int i=0;i<nn;i++)
	{
		*d++ = *s1 * *s1;
		s1++;
	}
}
static void divideSSE_float(Mat& src1, Mat& src2, Mat& dest)
{
	float* s1 = src1.ptr<float>(0);
	float* s2 = src2.ptr<float>(0);
	float* d = dest.ptr<float>(0);
	const int size = src1.size().area()/4;
	const int nn = src1.size().area()-size*4;
	for(int i=size;i--;)
	{
		__m128 ms1 = _mm_load_ps(s1);
		__m128 ms2 = _mm_load_ps(s2);
		ms1 = _mm_div_ps(ms1,ms2);
		_mm_store_ps(d,ms1);

		s1+=4,s2+=4,d+=4;
	}
	for(int i=0;i<nn;i++)
	{
		*d++ = *s1++ / *s2++;
	}
}

#ifdef FLOAT_BOX_FILTER_OFF
#define boxFilter2 boxFilter
#endif



static void guidedFilterSrc1Guidance3SSE_(const Mat& src, const Mat& guidance, Mat& dest, const int radius,const float eps)
{
	if(src.channels()!=1 && guidance.channels()!=3)
	{
		cout<<"Please input gray scale image as src, and color image as guidance."<<endl;
		return;
	}
	vector<Mat> I(3);
	vector<Mat> If(3);
	split(guidance,I);

	Mat temp;
	src.convertTo(temp,CV_32F,1.0/255);
	I[0].convertTo(If[0],CV_32F,1.0/255);
	I[1].convertTo(If[1],CV_32F,1.0/255);
	I[2].convertTo(If[2],CV_32F,1.0/255);

	const Size ksize(2*radius+1,2*radius+1);
	const Point PT(-1,-1);
	const float e=eps*eps;
	const Size imsize = src.size();
	const int size = src.size().area();
	const int ssesize = size/4;
	const int nn = size-ssesize*4;
	const double div = 1.0/ksize.area();

	Mat mean_I_r(imsize,CV_32F);
	Mat mean_I_g(imsize,CV_32F);
	Mat mean_I_b(imsize,CV_32F);
	Mat mean_p(imsize,CV_32F);

	//cout<<"mean computation"<<endl;
	boxFilter2(If[0],mean_I_r,CV_32F,ksize,PT,true,BORDER_TYPE);
	boxFilter2(If[1],mean_I_g,CV_32F,ksize,PT,true,BORDER_TYPE);
	boxFilter2(If[2],mean_I_b,CV_32F,ksize,PT,true,BORDER_TYPE);

	boxFilter2(temp,mean_p,CV_32F,ksize,PT,true,BORDER_TYPE);

	Mat mean_Ip_r(imsize,CV_32F);
	Mat mean_Ip_g(imsize,CV_32F);
	Mat mean_Ip_b(imsize,CV_32F);

	{
		float* s0 = temp.ptr<float>(0);
		float* s1 = If[0].ptr<float>(0);
		float* s2 = If[1].ptr<float>(0);
		float* s3 = If[2].ptr<float>(0);
		float* d1 = mean_Ip_r.ptr<float>(0);
		float* d2 = mean_Ip_g.ptr<float>(0);
		float* d3 = mean_Ip_b.ptr<float>(0);
		for(int i=ssesize;i--;)
		{
			const __m128 ms1 = _mm_load_ps(s0);
			__m128 ms2 = _mm_load_ps(s1);
			ms2 = _mm_mul_ps(ms1,ms2);
			_mm_store_ps(d1,ms2);

			ms2 = _mm_load_ps(s2);
			ms2 = _mm_mul_ps(ms1,ms2);
			_mm_store_ps(d2,ms2);

			ms2 = _mm_load_ps(s3);
			ms2 = _mm_mul_ps(ms1,ms2);
			_mm_store_ps(d3,ms2);

			s0+=4,s1+=4,s2+=4,s3+=4, d1+=4,d2+=4,d3+=4;
		}
		for(int i=0;i<nn;i++)
		{
			*d1 = *s0 * *s1;
			*d2 = *s0 * *s2;
			*d3 = *s0 * *s3;
			s0++,s1++,s2++,s3++, d1++,d2++,d3++;
		}
	}
	boxFilter2(mean_Ip_r,mean_Ip_r,CV_32F,ksize,PT,true,BORDER_TYPE);
	boxFilter2(mean_Ip_g,mean_Ip_g,CV_32F,ksize,PT,true,BORDER_TYPE);
	boxFilter2(mean_Ip_b,mean_Ip_b,CV_32F,ksize,PT,true,BORDER_TYPE);


	//temp: float src will not use;
	//mean_Ip_r,g,b will not use;
	//cout<<"covariance computation"<<endl;
	Mat cov_Ip_r=mean_Ip_r;
	Mat cov_Ip_g=mean_Ip_g;
	Mat cov_Ip_b=mean_Ip_b;

	{
		float* s0 = mean_p.ptr<float>(0);
		float* s1 = mean_I_r.ptr<float>(0);
		float* s2 = mean_I_g.ptr<float>(0);
		float* s3 = mean_I_b.ptr<float>(0);
		float* d1 = cov_Ip_r.ptr<float>(0);
		float* d2 = cov_Ip_g.ptr<float>(0);
		float* d3 = cov_Ip_b.ptr<float>(0);
		for(int i=ssesize;i--;)
		{
			const __m128 ms1 = _mm_load_ps(s0);

			__m128 ms2 = _mm_load_ps(s1);
			ms2 = _mm_mul_ps(ms1,ms2);
			__m128 ms3 = _mm_load_ps(d1);
			ms3 = _mm_sub_ps(ms3,ms2);
			_mm_store_ps(d1,ms3);

			ms2 = _mm_load_ps(s2);
			ms2 = _mm_mul_ps(ms1,ms2);
			ms3 = _mm_load_ps(d2);
			ms3 = _mm_sub_ps(ms3,ms2);
			_mm_store_ps(d2,ms3);

			ms2 = _mm_load_ps(s3);
			ms2 = _mm_mul_ps(ms1,ms2);
			ms3 = _mm_load_ps(d3);
			ms3 = _mm_sub_ps(ms3,ms2);
			_mm_store_ps(d3,ms3);

			s0+=4,s1+=4,s2+=4,s3+=4, d1+=4,d2+=4,d3+=4;
		}
		for(int i=0;i<nn;i++)
		{
			*d1 =  *d1 - ( *s0 * *s1);
			*d2 =  *d2 - ( *s0 * *s2);
			*d3 =  *d3 - ( *s0 * *s3);
			s0++,s1++,s2++,s3++, d1++,d2++,d3++;
		}
	}

	//cout<<"variance computation"<<endl;

	Mat var_I_rr;
	Mat var_I_rg;
	Mat var_I_rb;
	Mat var_I_gg;
	Mat var_I_gb;
	Mat var_I_bb;
	multiplySSE_float(If[0],temp);
	boxFilter2(temp,var_I_rr,CV_32F,ksize,PT,true,BORDER_TYPE);
	multiplySSE_float(If[0],If[1],temp);
	boxFilter2(temp,var_I_rg,CV_32F,ksize,PT,true,BORDER_TYPE);
	multiplySSE_float(If[0],If[2],temp);
	boxFilter2(temp,var_I_rb,CV_32F,ksize,PT,true,BORDER_TYPE);
	multiplySSE_float(If[1],temp);
	boxFilter2(temp,var_I_gg,CV_32F,ksize,PT,true,BORDER_TYPE);
	multiplySSE_float(If[1],If[2],temp);
	boxFilter2(temp,var_I_gb,CV_32F,ksize,PT,true,BORDER_TYPE);
	multiplySSE_float(If[2],temp);
	boxFilter2(temp,var_I_bb,CV_32F,ksize,PT,true,BORDER_TYPE);

	{
		float* s1 = mean_I_r.ptr<float>(0);
		float* s2 = mean_I_g.ptr<float>(0);
		float* s3 = mean_I_b.ptr<float>(0);
		float* d1 = var_I_rr.ptr<float>(0);
		float* d2 = var_I_rg.ptr<float>(0);
		float* d3 = var_I_rb.ptr<float>(0);
		float* d4 = var_I_gg.ptr<float>(0);
		float* d5 = var_I_gb.ptr<float>(0);
		float* d6 = var_I_bb.ptr<float>(0);
		const __m128 me = _mm_set1_ps(e);
		for(int i=ssesize;i--;)
		{
			const __m128 mr = _mm_load_ps(s1);

			__m128 md1 = _mm_load_ps(d1);
			__m128 ms1 = _mm_mul_ps(mr,mr);
			ms1 = _mm_sub_ps(md1,ms1);
			ms1 = _mm_add_ps(ms1,me);
			_mm_store_ps(d1,ms1);

			const __m128 mg = _mm_load_ps(s2);
			md1 = _mm_load_ps(d2);
			ms1 = _mm_mul_ps(mr,mg);
			ms1 = _mm_sub_ps(md1,ms1);
			_mm_store_ps(d2,ms1);

			const __m128 mb = _mm_load_ps(s3);
			md1 = _mm_load_ps(d3);
			ms1 = _mm_mul_ps(mr,mb);
			ms1 = _mm_sub_ps(md1,ms1);
			_mm_store_ps(d3,ms1);

			md1 = _mm_load_ps(d4);
			ms1 = _mm_mul_ps(mg,mg);
			ms1 = _mm_sub_ps(md1,ms1);
			ms1 = _mm_add_ps(ms1,me);
			_mm_store_ps(d4,ms1);

			md1 = _mm_load_ps(d5);
			ms1 = _mm_mul_ps(mg,mb);
			ms1 = _mm_sub_ps(md1,ms1);
			_mm_store_ps(d5,ms1);

			md1 = _mm_load_ps(d6);
			ms1 = _mm_mul_ps(mb,mb);
			ms1 = _mm_sub_ps(md1,ms1);
			ms1 = _mm_add_ps(ms1,me);
			_mm_store_ps(d6,ms1);


			s1+=4,s2+=4,s3+=4,
				d1+=4,d2+=4,d3+=4,d4+=4,d5+=4,d6+=4;
		}
		for(int i=0;i<nn;i++)
		{
			*d1 = *d1 - (*s1 * *s1)  + e;
			*d2 = *d2 - (*s1 * *s2)  ;
			*d3 = *d3 - (*s1 * *s3)  ;
			*d4 = *d4 - (*s2 * *s2)  + e;
			*d5 = *d5 - (*s2 * *s3)  ;
			*d6 = *d6 - (*s3 * *s3)  + e;
			s1++,s2++,s3++,	d1++,d2++,d3++,d4++,d5++,d6++;
		}
	}

	{
		float* rr = var_I_rr.ptr<float>(0);
		float* rg = var_I_rg.ptr<float>(0);
		float* rb = var_I_rb.ptr<float>(0);
		float* gg = var_I_gg.ptr<float>(0);
		float* gb = var_I_gb.ptr<float>(0);
		float* bb = var_I_bb.ptr<float>(0);

		float* covr = cov_Ip_r.ptr<float>(0);
		float* covg = cov_Ip_g.ptr<float>(0);
		float* covb = cov_Ip_b.ptr<float>(0);


		//float CV_DECL_ALIGNED(16) buf[4];
		//for(int i=ssesize;i--;rr+=4,rg+=4,rb+=4,gg+=4,gb+=4,bb+=4,covr+=4,covg+=4,covb+=4)
		for(int i=size;i--; rr++,rg++,rb++,gg++,gb++,bb++,covr++,covg++,covb++)
		{
			const float c0 = *gg * *bb - *gb * *gb;
			const float c1 = *rb * *gb - *rg * *bb;
			const float c2 = *rg * *gb - *rb * *gg;
			const float c4 = *rr * *bb - *rb * *rb;
			const float c5 = *rb * *rg - *rr * *gb;
			const float c8 = *rr * *gg - *rg * *rg;

			const float det = (*rr * *gg * *bb) + (*rg * *gb * *rb) + (*rb * *rg * *gb)  -(*rr * *gb * *gb) - (*rb * *gg * *rb) - (*rg * *rg* *bb);
			const float id = 1.f/det;

			
			const float r = *covr;
			const float g = *covg;
			const float b = *covb;

			*covr = id* (r*c0 + g*c1 + b*c2);
			*covg = id* (r*c1 + g*c4 + b*c5);
			*covb = id* (r*c2 + g*c5 + b*c8);

			//SSE4 vc2010 make faster code... 
			/*const __m128 v = _mm_set_ps(r,g,b,0.f);
			const __m128 v2 = _mm_set1_ps(id);
			__m128 a = _mm_mul_ps(v2,_mm_add_ps(_mm_add_ps(_mm_dp_ps(v, _mm_set_ps(c0,c1,c2,0.f),225),_mm_dp_ps(v, _mm_set_ps(c1,c4,c5,0.f),226)),_mm_dp_ps(v, _mm_set_ps(c2,c5,c8,0.f),228)));
			_mm_store_ps(buf,a);

			*covr = buf[0];
			*covg = buf[1];
			*covb = buf[2];*/
			
			
			
			

			//over flow...
			/*
			__m128 mrr = _mm_load_ps(rr);
			__m128 mrg = _mm_load_ps(rg);
			__m128 mrb = _mm_load_ps(rb);
			__m128 mgg = _mm_load_ps(gg);
			__m128 mgb = _mm_load_ps(gb);
			__m128 mbb = _mm_load_ps(bb);

			__m128 ggbb = _mm_mul_ps(mgg,mbb);
			__m128 gbgb = _mm_mul_ps(mgb,mgb);
			//float c0 = *gg * *bb - *gb * *gb;
			__m128 mc0 = _mm_sub_ps(ggbb,gbgb);

			__m128 rbgb = _mm_mul_ps(mrb,mgb);
			__m128 rgbb = _mm_mul_ps(mrg,mbb);
			//float c1 = *rb * *gb - *rg * *bb;
			__m128 mc1 = _mm_sub_ps(rbgb,rgbb);

			__m128 rggb = _mm_mul_ps(mrg,mgb);
			__m128 rbgg = _mm_mul_ps(mrb,mgg);
			//float c2 = *rg * *gb - *rb * *gg;
			__m128 mc2 = _mm_sub_ps(rbgb,rbgg);

			__m128 rrbb = _mm_mul_ps(mrr,mbb);
			__m128 rbrb = _mm_mul_ps(mrb,mrb);
			//float c4 = *rr * *bb - *rb * *rb;
			__m128 mc4 = _mm_sub_ps(rrbb,rbrb);

			__m128 rbrg = _mm_mul_ps(mrb,mrg);
			__m128 rrgb = _mm_mul_ps(mrr,mgb);
			//float c5 = *rb * *rg - *rr * *gb;
			__m128 mc5 = _mm_sub_ps(rbrg,rrgb);

			__m128 rrgg = _mm_mul_ps(mrr,mgg);
			__m128 rgrg = _mm_mul_ps(mrg,mrg);
			//float c8 = *rr * *gg - *rg * *rg;
			__m128 mc8 = _mm_sub_ps(rrgg,rgrg);

			//__m128 m1 = _mm_set1_ps(1.0f);
			const __m128 iv = _mm_sub_ps(_mm_add_ps(_mm_add_ps(_mm_mul_ps(rrgg,mbb),_mm_mul_ps(rggb,mrb)),_mm_mul_ps(rbrg,mgb)),
			_mm_add_ps(_mm_add_ps(_mm_mul_ps(rrgb,mgb),_mm_mul_ps(rbgg,mrb)),_mm_mul_ps(rgrg,mbb)));
			//const float det = (*rr * *gg * *bb) + (*rg * *gb * *rb) + (*rb * *rg * *gb)  -(*rr * *gb * *gb) - (*rb * *gg * *rb) - (*rg * *rg* *bb);
			//const float id = 1.f/det;

			//const float r = *covr;
			//const float g = *covg;
			//const float b = *covb;
			const __m128 mcvr = _mm_load_ps(covr);
			const __m128 mcvg = _mm_load_ps(covg);
			const __m128 mcvb = _mm_load_ps(covb);

			mrr = _mm_div_ps(_mm_add_ps(_mm_add_ps(_mm_mul_ps(mcvr,mc0),_mm_mul_ps(mcvg,mc1)),_mm_mul_ps(mcvb,mc2)),iv);
			mrg = _mm_div_ps(_mm_add_ps(_mm_add_ps(_mm_mul_ps(mcvr,mc1),_mm_mul_ps(mcvg,mc4)),_mm_mul_ps(mcvb,mc5)),iv);
			mrb = _mm_div_ps(_mm_add_ps(_mm_add_ps(_mm_mul_ps(mcvr,mc2),_mm_mul_ps(mcvg,mc5)),_mm_mul_ps(mcvb,mc8)),iv);
			//*covr = id* (r*c0 + g*c1 + b*c2);
			//*covg = id* (r*c1 + g*c4 + b*c5);
			//*covb = id* (r*c2 + g*c5 + b*c8);

			_mm_store_ps(covr,mrr);
			_mm_store_ps(covg,mrg);
			_mm_store_ps(covb,mrb);
			*/
		}
	}

	Mat a_r =cov_Ip_r;
	Mat a_g =cov_Ip_g;
	Mat a_b =cov_Ip_b;

	{
		float* s0 = mean_p.ptr<float>(0);
		float* s1 = mean_I_r.ptr<float>(0);
		float* s2 = mean_I_g.ptr<float>(0);
		float* s3 = mean_I_b.ptr<float>(0);
		float* a1 = a_r.ptr<float>(0);
		float* a2 = a_g.ptr<float>(0);
		float* a3 = a_b.ptr<float>(0);
		for(int i=ssesize;i--;)
		{
			__m128 ms3 = _mm_load_ps(s0);

			__m128 ms1 = _mm_load_ps(s1);
			__m128 ms2 = _mm_load_ps(a1);
			ms1 = _mm_mul_ps(ms2,ms1);
			ms3 = _mm_sub_ps(ms3,ms1);

			ms1 = _mm_load_ps(s2);
			ms2 = _mm_load_ps(a2);
			ms1 = _mm_mul_ps(ms2,ms1);
			ms3 = _mm_sub_ps(ms3,ms1);

			ms1 = _mm_load_ps(s3);
			ms2 = _mm_load_ps(a3);
			ms1 = _mm_mul_ps(ms2,ms1);
			ms3 = _mm_sub_ps(ms3,ms1);

			_mm_store_ps(s0,ms3);

			s0+=4,s1+=4,s2+=4,s3+=4, a1+=4,a2+=4,a3+=4;
		}
		for(int i=0;i<nn;i++)
		{
			*s0 = *s0 - (*a1 * *s1) - ((*a2 * *s2)) -(*a3 * *s3);
			s0++,s1++,s2++,s3++, a1++,a2++,a3++;
		}
	}

	Mat b = mean_p;
	boxFilter2(a_r,a_r,CV_32F,ksize,PT,true,BORDER_TYPE);//break a_r
	boxFilter2(a_g,a_g,CV_32F,ksize,PT,true,BORDER_TYPE);//break a_g
	boxFilter2(a_b,a_b,CV_32F,ksize,PT,true,BORDER_TYPE);//break a_b
	boxFilter2(b,temp,CV_32F,ksize,PT,true,BORDER_TYPE);

	{
		float* s0 = temp.ptr<float>(0);
		float* s1 = If[0].ptr<float>(0);
		float* s2 = If[1].ptr<float>(0);
		float* s3 = If[2].ptr<float>(0);
		float* d1 = a_r.ptr<float>(0);
		float* d2 = a_g.ptr<float>(0);
		float* d3 = a_b.ptr<float>(0);
		const __m128 me = _mm_set1_ps(255.f);
		for(int i=ssesize;i--;)
		{
			__m128 ms1 = _mm_load_ps(s0);

			__m128 ms2 = _mm_load_ps(s1);
			__m128 ms3 = _mm_load_ps(d1);
			ms2 = _mm_mul_ps(ms2,ms3);
			ms1 = _mm_add_ps(ms1,ms2);

			ms2 = _mm_load_ps(s2);
			ms3 = _mm_load_ps(d2);
			ms2 = _mm_mul_ps(ms2,ms3);
			ms1 = _mm_add_ps(ms1,ms2);

			ms2 = _mm_load_ps(s3);
			ms3 = _mm_load_ps(d3);
			ms2 = _mm_mul_ps(ms2,ms3);
			ms1 = _mm_add_ps(ms1,ms2);

			ms1 = _mm_mul_ps(ms1,me);
			_mm_store_ps(s0,ms1);

			s0+=4,s1+=4,s2+=4,s3+=4, d1+=4,d2+=4,d3+=4;
		}
		for(int i=0;i<nn;i++)
		{
			*s0 = 255.f* (*s0 + (*s1 * *d1) + (*s2 * *d2) + (*s3 * *d3));
			s0++,s1++,s2++,s3++, d1++,d2++,d3++;
		}
	}
	temp.convertTo(dest,src.type());
}

static void cvt8u32f(const Mat& src, Mat& dest, const float amp)
{
	const int imsize = src.size().area()/8;
	const int nn = src.size().area()- imsize*8 ;
	uchar* s = (uchar*)src.ptr(0);
	float* d = dest.ptr<float>(0);
	const __m128 mamp = _mm_set_ps1(amp);
	const __m128i zero = _mm_setzero_si128();
	for(int i=imsize;i--;)
	{
		__m128i s1 = _mm_loadl_epi64((__m128i*)s);

		_mm_store_ps(d,_mm_mul_ps(mamp,_mm_cvtepi32_ps(_mm_cvtepu8_epi32(s1))));
		_mm_store_ps(d+4,_mm_mul_ps(mamp,_mm_cvtepi32_ps(_mm_cvtepu8_epi32(_mm_srli_si128(s1,4)))));
		s+=8;
		d+=8;
	}
	for(int i=0;i<nn;i++)
	{
		*d = (float)*s * amp;
		s++,d++;
	}

}
static void guidedFilterSrc1Guidance3_(const Mat& src, const Mat& guidance, Mat& dest, const int radius,const float eps)
{
	if(src.channels()!=1 && guidance.channels()!=3)
	{
		cout<<"Please input gray scale image as src, and color image as guidance."<<endl;
		return;
	}
	vector<Mat> I(3);
	vector<Mat> If(3);
	split(guidance,I);

	const Size ksize(2*radius+1,2*radius+1);
	const Point PT(-1,-1);
	const float e=eps*eps;
	const Size imsize = src.size();
	const int size = src.size().area();
	const double div = 1.0/ksize.area();

	Mat temp(imsize,CV_32F);
	If[0].create(imsize,CV_32F);
	If[1].create(imsize,CV_32F);
	If[2].create(imsize,CV_32F);

	if(src.type()==CV_8U)
	{
		cvt8u32f( src,temp,1.f/255.f);
		cvt8u32f( I[0],If[0],1.f/255.f);
		cvt8u32f( I[1],If[1],1.f/255.f);
		cvt8u32f( I[2],If[2],1.f/255.f);
	}
	else
	{
		src.convertTo(temp,CV_32F,1.f/255.f);
		I[0].convertTo(If[0],CV_32F,1.f/255.f);
		I[1].convertTo(If[1],CV_32F,1.f/255.f);
		I[2].convertTo(If[2],CV_32F,1.f/255.f);
	}
	Mat mean_I_r(imsize,CV_32F);
	Mat mean_I_g(imsize,CV_32F);
	Mat mean_I_b(imsize,CV_32F);
	Mat mean_p(imsize,CV_32F);

	//cout<<"mean computation"<<endl;
	boxFilter(If[0],mean_I_r,CV_32F,ksize,PT,true,BORDER_TYPE);
	boxFilter(If[1],mean_I_g,CV_32F,ksize,PT,true,BORDER_TYPE);
	boxFilter(If[2],mean_I_b,CV_32F,ksize,PT,true,BORDER_TYPE);

	boxFilter(temp,mean_p,CV_32F,ksize,PT,true,BORDER_TYPE);

	Mat mean_Ip_r(imsize,CV_32F);
	Mat mean_Ip_g(imsize,CV_32F);
	Mat mean_Ip_b(imsize,CV_32F);
	multiply(If[0],temp,mean_Ip_r);//Ir*p
	boxFilter(mean_Ip_r,mean_Ip_r,CV_32F,ksize,PT,true,BORDER_TYPE);
	multiply(If[1],temp,mean_Ip_g);//Ig*p
	boxFilter(mean_Ip_g,mean_Ip_g,CV_32F,ksize,PT,true,BORDER_TYPE);
	multiply(If[2],temp,mean_Ip_b);//Ib*p
	boxFilter(mean_Ip_b,mean_Ip_b,CV_32F,ksize,PT,true,BORDER_TYPE);

	//temp: float src will not use;
	//mean_Ip_r,g,b will not use;
	//cout<<"covariance computation"<<endl;
	Mat cov_Ip_r=mean_Ip_r;
	Mat cov_Ip_g=mean_Ip_g;
	Mat cov_Ip_b=mean_Ip_b;
	multiply(mean_I_r,mean_p,temp);
	cov_Ip_r -= temp;
	multiply(mean_I_g,mean_p,temp);
	cov_Ip_g -= temp;
	multiply(mean_I_b,mean_p,temp);
	cov_Ip_b -= temp;

	//cout<<"variance computation"<<endl;

	//getCovImage();
	//�R�R����X�v���b�g
	Mat var_I_rr;
	Mat var_I_rg;
	Mat var_I_rb;
	Mat var_I_gg;
	Mat var_I_gb;
	Mat var_I_bb;
	multiply(If[0],If[0],temp);
	boxFilter(temp,var_I_rr,CV_32F,ksize,PT,true,BORDER_TYPE);
	multiply(mean_I_r,mean_I_r,temp);
	var_I_rr-=temp;

	multiply(If[0],If[1],temp);
	boxFilter(temp,var_I_rg,CV_32F,ksize,PT,true,BORDER_TYPE);
	multiply(mean_I_r,mean_I_g,temp);
	var_I_rg-=temp;

	multiply(If[0],If[2],temp);
	boxFilter(temp,var_I_rb,CV_32F,ksize,PT,true,BORDER_TYPE);
	multiply(mean_I_r,mean_I_b,temp);
	var_I_rb-=temp;

	multiply(If[1],If[1],temp);
	boxFilter(temp,var_I_gg,CV_32F,ksize,PT,true,BORDER_TYPE);
	multiply(mean_I_g,mean_I_g,temp);
	var_I_gg-=temp;

	multiply(If[1],If[2],temp);
	boxFilter(temp,var_I_gb,CV_32F,ksize,PT,true,BORDER_TYPE);
	multiply(mean_I_g,mean_I_b,temp);
	var_I_gb-=temp;

	multiply(If[2],If[2],temp);
	boxFilter(temp,var_I_bb,CV_32F,ksize,PT,true,BORDER_TYPE);
	multiply(mean_I_b,mean_I_b,temp);
	var_I_bb-=temp;

	var_I_rr+=e;
	var_I_gg+=e;
	var_I_bb+=e;

	float* rr = var_I_rr.ptr<float>(0);
	float* rg = var_I_rg.ptr<float>(0);
	float* rb = var_I_rb.ptr<float>(0);
	float* gg = var_I_gg.ptr<float>(0);
	float* gb = var_I_gb.ptr<float>(0);
	float* bb = var_I_bb.ptr<float>(0);

	float* covr = cov_Ip_r.ptr<float>(0);
	float* covg = cov_Ip_g.ptr<float>(0);
	float* covb = cov_Ip_b.ptr<float>(0);

	{
		//CalcTime t("cov");
		for(int i=size;i--; rr++,rg++,rb++,gg++,gb++,bb++,covr++,covg++,covb++)
		{/*
		 Matx33f sigmaEps
		 (
		 *rr,*rg,*rb,
		 *rg,*gg,*gb,
		 *rb,*gb,*bb
		 );
		 Matx33f inv = sigmaEps.inv(cv::DECOMP_LU);


		 const float r = *covr;
		 const float g = *covg;
		 const float b = *covb;
		 *covr = r*inv(0,0) + g*inv(1,0) + b*inv(2,0);
		 *covg = r*inv(0,1) + g*inv(1,1) + b*inv(2,1);
		 *covb = r*inv(0,2) + g*inv(1,2) + b*inv(2,2);*/

			const float det = (*rr * *gg * *bb) + (*rg * *gb * *rb) + (*rb * *rg * *gb)  -(*rr * *gb * *gb) - (*rb * *gg * *rb) - (*rg * *rg* *bb);
			const float id = 1.f/det;

			float c0 = *gg * *bb - *gb * *gb;
			float c1 = *rb * *gb - *rg * *bb;
			float c2 = *rg * *gb - *rb * *gg;
			float c4 = *rr * *bb - *rb * *rb;
			float c5 = *rb * *rg - *rr * *gb;
			float c8 = *rr * *gg - *rg * *rg;
			const float r = *covr;
			const float g = *covg;
			const float b = *covb;
			*covr = id* (r*c0 + g*c1 + b*c2);
			*covg = id* (r*c1 + g*c4 + b*c5);
			*covb = id* (r*c2 + g*c5 + b*c8);

			/*cout<<format("%f %f %f \n %f %f %f \n %f %f %f \n",
			id*(*gg * *bb - *gb * *gb),//c0
			id*(*rb * *gb - *rg * *bb),//c1
			id*(*rg * *gb - *rb * *gg),//c2
			id*(*gb * *rb - *rg * *bb),//c3=c1
			id*(*rr * *bb - *rb * *rb),//c4
			id*(*rb * *rg - *rr * *gb),//c5
			id*(*rg * *gb - *rb * *gg),//c6=c2
			id*(*rb * *rg - *rr * *gb),//c7 = c5
			id*(*rr * *gg - *rg * *rg)//c8
			);
			cout<<determinant(sigmaEps)<<endl;
			cout<<det<<endl;
			cout<<Mat(inv)<<endl;
			getchar();*/
		}
	}

	Mat a_r =cov_Ip_r;
	Mat a_g =cov_Ip_g;
	Mat a_b =cov_Ip_b;

	multiply(a_r,mean_I_r,mean_I_r);//break mean_I_r;
	multiply(a_g,mean_I_g,mean_I_g);//break mean_I_g;
	multiply(a_b,mean_I_b,mean_I_b);//break mean_I_b;
	mean_p -= (mean_I_r+ mean_I_g + mean_I_b);
	Mat b = mean_p;

	boxFilter(a_r,a_r,CV_32F,ksize,PT,true,BORDER_TYPE);//break a_r
	boxFilter(a_g,a_g,CV_32F,ksize,PT,true,BORDER_TYPE);//break a_g
	boxFilter(a_b,a_b,CV_32F,ksize,PT,true,BORDER_TYPE);//break a_b

	boxFilter(b,temp,CV_32F,ksize,PT,true,BORDER_TYPE);
	multiply(a_r,If[0],a_r);
	multiply(a_g,If[1],a_g);
	multiply(a_b,If[2],a_b);
	temp += (a_r + a_g + a_b);

	temp.convertTo(dest,src.type(),255.0);
}

static void guidedFilterSrc1Guidance1SSE_(const Mat& src, const Mat& joint, Mat& dest,const int radius,const float eps)
{	
	if(src.channels()!=1 && joint.channels()!=1)
	{
		cout<<"Please input gray scale image."<<endl;
		return;
	}
	if(dest.empty())dest.create(src.size(),src.type());
	const Size ksize(2*radius+1,2*radius+1);
	const Size imsize = src.size();
	const int ssesize = imsize.area()/4; //const int ssesize = 0;
	const int nn = imsize.area()-ssesize*4;
	const float e=eps*eps;

	STATICMAT x1(imsize,CV_32F),x2(imsize,CV_32F),x3(imsize,CV_32F);
	STATICMAT mJoint(imsize,CV_32F);//mean_I
	STATICMAT mSrc(imsize,CV_32F);//mean_p

	STATICMAT sf(imsize,CV_32F);
	STATICMAT jf(imsize,CV_32F);
	if(src.type()==CV_8U)
	{
		cvt8u32f(src,sf,1.f/255.f);
		cvt8u32f(joint,jf,1.f/255.f);
	}
	else
	{
		src.convertTo(sf,CV_32F,1.0/255.0);
		joint.convertTo(jf,CV_32F,1.0/255.0);
	}



	{
		float* s1 = jf.ptr<float>(0);
		float* s2 = sf.ptr<float>(0);
		float* d = x2.ptr<float>(0);
		float* d2 = x1.ptr<float>(0);

		for(int i=ssesize;i--;)
		{
			__m128 ms1 = _mm_load_ps(s1);
			__m128 ms2 = _mm_load_ps(s2);
			ms2 = _mm_mul_ps(ms1,ms2);
			_mm_store_ps(d,ms2);

			ms1 = _mm_mul_ps(ms1,ms1);
			_mm_store_ps(d2,ms1);
			s1+=4,s2+=4,d+=4,d2+=4;
		}
		for(int i=0;i<nn;i++)
		{
			*d2++ = *s1 * *s1;
			*d++ = *s1++ * *s2++;
		}
	}


	boxFilter2(jf,mJoint,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//mJoint*K
	boxFilter2(sf,mSrc,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//mSrc*K
	boxFilter2(x2,x3,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x3*K
	boxFilter2(x1,x2,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x2*K	
	{
		float* s1=mJoint.ptr<float>(0);
		float* s2=x2.ptr<float>(0);
		float* s3=x3.ptr<float>(0);//*s1 = *s3 - *s1 * *s5;
		float* s4=x1.ptr<float>(0);
		float* s5=mSrc.ptr<float>(0);
		const __m128 ms4 = _mm_set1_ps(e);
		for(int i=ssesize;i--;)
		{
			//mjoint*mjoint
			const __m128 ms1 = _mm_load_ps(s1);
			const __m128 ms5 = _mm_load_ps(s5);
			__m128 ms2 = _mm_mul_ps(ms1,ms1);
			//x2-x1+e
			__m128 ms3 = _mm_load_ps(s2);
			ms2 = _mm_sub_ps(ms3,ms2);
			ms2 = _mm_add_ps(ms2,ms4);
			//x3/xx
			ms3 = _mm_load_ps(s3);
			ms3 = _mm_sub_ps(ms3,_mm_mul_ps(ms1,ms5));

			ms2 = _mm_div_ps(ms3,ms2);
			_mm_store_ps(s3,ms2);
			//ms
			ms2 = _mm_mul_ps(ms2,ms1);
			ms3 = _mm_sub_ps(ms2,ms5);
			_mm_store_ps(s4,ms3);

			s1+=4,s2+=4,s3+=4,s4+=4,s5+=4;
		}
		for(int i=0;i<nn;i++)
		{
			*s3 = (*s3 - (*s1 * *s5))   / (*s2 - (*s1 * *s1) + e);
			*s4 = (*s3 * *s1) - *s5;
			s1++,s2++,s3++,s4++,s5++;
		}
	}
	boxFilter2(x3,x2,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x2*k
	boxFilter2(x1,x3,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x3*k
	{
		float* s1=x2.ptr<float>(0);
		float* s2=jf.ptr<float>(0);
		float* s3=x3.ptr<float>(0);

		const __m128 ms3 = _mm_set1_ps(255.f);
		for(int i=ssesize;i--;)
		{
			__m128 ms1 = _mm_load_ps(s1);
			__m128 ms2 = _mm_load_ps(s2);
			ms1 = _mm_mul_ps(ms1,ms2);

			ms2 = _mm_load_ps(s3);
			ms1 = _mm_sub_ps(ms1,ms2);

			ms2 = _mm_mul_ps(ms1,ms3);
			_mm_store_ps(s1,ms2);

			s1+=4,s2+=4,s3+=4;
		}
		for(int i=0;i<nn;i++)
		{
			*s1 = 255.f * ((*s1 * *s2) - * s3 );
			s1++,s2++,s3++;
		}
	}
	x2.convertTo(dest,src.type());
}

static void guidedFilterSrc1Guidance1_(const Mat& src, const Mat& joint,Mat& dest,const int radius,const float eps)
{	
	if(src.channels()!=1 && joint.channels()!=1)
	{
		cout<<"Please input gray scale image."<<endl;
		return;
	}
	//some opt
	Size ksize(2*radius+1,2*radius+1);
	Size imsize = src.size();
	const float e=eps*eps;

	Mat sf;src.convertTo(sf,CV_32F,1.0/255);
	Mat jf;joint.convertTo(jf,CV_32F,1.0/255);////////////////
	Mat mJoint(imsize,CV_32F);//mean_I///////////////////////////////
	Mat mSrc(imsize,CV_32F);//mean_p

	boxFilter(jf,mJoint,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//mJoint*K/////////////////////////
	boxFilter(sf,mSrc,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//mSrc*K

	Mat x1(imsize,CV_32F),x2(imsize,CV_32F),x3(imsize,CV_32F);

	multiply(jf,sf,x1);//x1*1
	boxFilter(x1,x3,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x3*K
	multiply(mJoint,mSrc,x1);//;x1*K*K
	x3-=x1;//x1 div k ->x3*k
	multiply(jf,jf,x1);////////////////////////////////////
	boxFilter(x1,x2,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x2*K
	multiply(mJoint,mJoint,x1);//x1*K*K
	sf = Mat(x2 - x1)+e;
	divide(x3,sf,x3);
	multiply(x3,mJoint,x1);
	x1-=mSrc;
	boxFilter(x3,x2,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x2*k
	boxFilter(x1,x3,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x3*k
	multiply(x2,jf,x1);//x1*K
	Mat temp = x1-x3;//
	temp.convertTo(dest,src.type(),255);
}

void guidedFilterRGBSplit(Mat& src, Mat& guidance,Mat& dest,const int radius,const float eps)
{
	if(src.channels()==3 && guidance.channels()==3)
	{
		vector<Mat> v;
		vector<Mat> d(3);
		vector<Mat> j;
		split(src,v);
		split(guidance,j);
		guidedFilterSrc1Guidance1_(v[0],j[0],d[0],radius, eps);
		guidedFilterSrc1Guidance1_(v[1],j[0],d[1],radius, eps);
		guidedFilterSrc1Guidance1_(v[2],j[0],d[2],radius, eps);
		merge(d,dest);
	}
}

static void weightedBoxFilter(Mat& src, Mat& weight, Mat& dest,int type, Size ksize,Point pt,int border_type)
{
	Mat sw;
	Mat wsrc(src.size(),CV_32F);
	boxFilter(weight,sw,CV_32F,ksize,pt,true,border_type);
	multiplySSE_float(src,weight,wsrc);//sf*sf
	boxFilter(wsrc,dest,CV_32F,ksize,pt,true,border_type);
	divideSSE_float(dest,sw,dest);
}
void weightedAdaptiveGuidedFilter(Mat& src, Mat& guidance,Mat& guidance2, Mat& weight,Mat& dest, const int radius,const float eps)
{
	if(src.channels()!=1 && guidance.channels()!=1)
	{
		cout<<"Please input gray scale image."<<endl;
		return;
	}
	//some opt
	Size ksize(2*radius+1,2*radius+1);
	Size imsize = src.size();
	const float e=eps*eps;

	Mat sf;src.convertTo(sf,CV_32F,1.0/255);
	Mat jf;guidance.convertTo(jf,CV_32F,1.0/255);
	Mat mJoint(imsize,CV_32F);//mean_I
	Mat mSrc(imsize,CV_32F);//mean_p

	weightedBoxFilter(jf,weight,mJoint,CV_32F,ksize,Point(-1,-1),BORDER_TYPE);//mJoint*K
	weightedBoxFilter(sf,weight,mSrc,CV_32F,ksize,Point(-1,-1),BORDER_TYPE);//mSrc*K

	Mat x1(imsize,CV_32F),x2(imsize,CV_32F),x3(imsize,CV_32F);

	multiplySSE_float(jf,sf,x1);//x1*1
	weightedBoxFilter(x1,weight,x3,CV_32F,ksize,Point(-1,-1),BORDER_TYPE);//x3*K
	multiplySSE_float(mJoint,mSrc,x1);//;x1*K*K
	x3-=x1;//x1 div k ->x3*k
	multiplySSE_float(jf,x1);
	weightedBoxFilter(x1,weight,x2,CV_32F,ksize,Point(-1,-1),BORDER_TYPE);//x2*K
	multiplySSE_float(mJoint,x1);//x1*K*K
	sf = Mat(x2 - x1)+e;
	divideSSE_float(x3,sf,x3);
	multiplySSE_float(x3,mJoint,x1);
	x1-=mSrc;
	boxFilter(x3,x2,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x2*k
	boxFilter(x1,x3,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x3*k

	guidance2.convertTo(jf,CV_32F,1.0/255);
	multiplySSE_float(x2,jf,x1);//x1*K
	Mat temp = x1-x3;//
	temp.convertTo(dest,src.type(),255);
}

void weightedGuidedFilter(Mat& src, Mat& guidance, Mat& weight,Mat& dest, const int radius,const float eps)
{

	if(src.channels()!=1 && guidance.channels()!=1)
	{
		cout<<"Please input gray scale image."<<endl;
		return;
	}
	//some opt
	Size ksize(2*radius+1,2*radius+1);
	Size imsize = src.size();
	const float e=eps*eps;

	Mat sf;src.convertTo(sf,CV_32F,1.0/255);
	Mat jf;guidance.convertTo(jf,CV_32F,1.0/255);
	Mat mJoint(imsize,CV_32F);//mean_I
	Mat mSrc(imsize,CV_32F);//mean_p

	weightedBoxFilter(jf,weight,mJoint,CV_32F,ksize,Point(-1,-1),BORDER_TYPE);//mJoint*K
	weightedBoxFilter(sf,weight,mSrc,CV_32F,ksize,Point(-1,-1),BORDER_TYPE);//mSrc*K

	Mat x1(imsize,CV_32F),x2(imsize,CV_32F),x3(imsize,CV_32F);

	multiplySSE_float(jf,sf,x1);//x1*1
	weightedBoxFilter(x1,weight,x3,CV_32F,ksize,Point(-1,-1),BORDER_TYPE);//x3*K
	multiplySSE_float(mJoint,mSrc,x1);//;x1*K*K
	x3-=x1;//x1 div k ->x3*k
	multiplySSE_float(jf,x1);
	weightedBoxFilter(x1,weight,x2,CV_32F,ksize,Point(-1,-1),BORDER_TYPE);//x2*K
	multiplySSE_float(mJoint,x1);//x1*K*K
	sf = Mat(x2 - x1)+e;
	divideSSE_float(x3,sf,x3);
	multiplySSE_float(x3,mJoint,x1);
	x1-=mSrc;
	boxFilter(x3,x2,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x2*k
	boxFilter(x1,x3,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x3*k
	multiplySSE_float(x2,jf,x1);//x1*K
	Mat temp = x1-x3;//
	temp.convertTo(dest,src.type(),255);


	/*	if(src.channels()!=1 && guidance.channels()!=1)
	{
	cout<<"Please input gray scale image."<<endl;
	return;
	}
	//some opt
	Size ksize(2*radius+1,2*radius+1);
	Size imsize = src.size();
	const float e=eps*eps;

	Mat sf;src.convertTo(sf,CV_32F,1.0/255);
	Mat jf;guidance.convertTo(jf,CV_32F,1.0/255);
	Mat mJoint(imsize,CV_32F);//mean_I
	Mat mSrc(imsize,CV_32F);//mean_p

	boxFilter(jf,mJoint,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//mJoint*K
	boxFilter(sf,mSrc,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//mSrc*K

	Mat x1(imsize,CV_32F),x2(imsize,CV_32F),x3(imsize,CV_32F);

	multiplySSE_float(jf,sf,x1);//x1*1
	boxFilter(x1,x3,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x3*K
	multiplySSE_float(mJoint,mSrc,x1);//;x1*K*K
	x3-=x1;//x1 div k ->x3*k
	multiplySSE_float(jf,x1);
	boxFilter(x1,x2,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x2*K
	multiplySSE_float(mJoint,x1);//x1*K*K
	sf = Mat(x2 - x1)+e;
	divideSSE_float(x3,sf,x3);
	multiplySSE_float(x3,mJoint,x1);
	x1-=mSrc;
	weightedBoxFilter(x3,weight,x2,CV_32F,ksize,Point(-1,-1),BORDER_TYPE);//x2*k
	weightedBoxFilter(x1,weight,x3,CV_32F,ksize,Point(-1,-1),BORDER_TYPE);//x3*k
	multiplySSE_float(x2,jf,x1);//x1*K
	Mat temp = x1-x3;//
	temp.convertTo(dest,src.type(),255);*/
}

void weightedGuidedFilter2(Mat& src, Mat& guidance, Mat& weight,Mat& dest, const int radius,const float eps)
{

	if(src.channels()!=1 && guidance.channels()!=1)
	{
		cout<<"Please input gray scale image."<<endl;
		return;
	}
	//some opt
	Size ksize(2*radius+1,2*radius+1);
	Size imsize = src.size();
	const float e=eps*eps;

	Mat sf;src.convertTo(sf,CV_32F,1.0/255);
	Mat jf;guidance.convertTo(jf,CV_32F,1.0/255);
	Mat mJoint(imsize,CV_32F);//mean_I
	Mat mSrc(imsize,CV_32F);//mean_p

	boxFilter(jf,mJoint,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//mJoint*K
	boxFilter(sf,mSrc,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//mSrc*K

	Mat x1(imsize,CV_32F),x2(imsize,CV_32F),x3(imsize,CV_32F);

	multiplySSE_float(jf,sf,x1);//x1*1
	boxFilter(x1,x3,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x3*K
	multiplySSE_float(mJoint,mSrc,x1);//;x1*K*K
	x3-=x1;//x1 div k ->x3*k
	multiplySSE_float(jf,x1);
	boxFilter(x1,x2,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x2*K
	multiplySSE_float(mJoint,x1);//x1*K*K
	sf = Mat(x2 - x1)+e;
	divideSSE_float(x3,sf,x3);
	multiplySSE_float(x3,mJoint,x1);
	x1-=mSrc;
	weightedBoxFilter(x3,weight,x2,CV_32F,ksize,Point(-1,-1),BORDER_TYPE);//x2*k
	weightedBoxFilter(x1,weight,x3,CV_32F,ksize,Point(-1,-1),BORDER_TYPE);//x3*k
	multiplySSE_float(x2,jf,x1);//x1*K
	Mat temp = x1-x3;//
	temp.convertTo(dest,src.type(),255);
}

void guidedFilterBF(const Mat& src, const Mat& guidance, Mat& dest, const int radius,const float eps)
{
	if(src.channels()==1 && guidance.channels()==3)
	{
		guidedFilterSrc1Guidance3_(src,guidance,dest,radius,eps);
	}
	else if(src.channels()==1 && guidance.channels()==1)
	{
		guidedFilterSrc1Guidance1_(src,guidance,dest,radius,eps);
	}
	else if(src.channels()==3 && guidance.channels()==3)
	{
		vector<Mat> v;
		vector<Mat> d(3);
		split(src,v);

		guidedFilterSrc1Guidance3_(v[0],guidance,d[0],radius, eps);
		guidedFilterSrc1Guidance3_(v[1],guidance,d[1],radius, eps);
		guidedFilterSrc1Guidance3_(v[2],guidance,d[2],radius, eps);

		merge(d,dest);
	}
	else if(src.channels()==3 && guidance.channels()==1)
	{
		vector<Mat> v;
		vector<Mat> d(3);
		split(src,v);

		guidedFilterSrc1Guidance1_(v[0],guidance,d[0],radius, eps);
		guidedFilterSrc1Guidance1_(v[1],guidance,d[1],radius, eps);
		guidedFilterSrc1Guidance1_(v[2],guidance,d[2],radius, eps);

		merge(d,dest);
	}
}
void guidedFilter(const Mat& src, const Mat& guidance, Mat& dest, const int radius,const float eps)
{
	if(radius==0){src.copyTo(dest);return;}
	bool sse = checkHardwareSupport(CV_CPU_SSE2);
	
	if(src.channels()==1 && guidance.channels()==3)
	{
		
		if(sse)
			guidedFilterSrc1Guidance3SSE_(src,guidance,dest,radius,eps);
		else
			guidedFilterSrc1Guidance3_(src,guidance,dest,radius,eps);
	}
	else if(src.channels()==1 && guidance.channels()==1)
	{
		if(sse)
		{
			guidedFilterSrc1Guidance1SSE_(src,guidance,dest,radius,eps);
		}
		else
		{
			guidedFilterSrc1Guidance1_(src,guidance,dest,radius,eps);
		}
	}
	else if(src.channels()==3 && guidance.channels()==3)
	{

		vector<Mat> v;
		vector<Mat> d(3);
		split(src,v);
		if(sse)
		{
			guidedFilterSrc1Guidance3SSE_(v[0],guidance,d[0],radius, eps);
			guidedFilterSrc1Guidance3SSE_(v[1],guidance,d[1],radius, eps);
			guidedFilterSrc1Guidance3SSE_(v[2],guidance,d[2],radius, eps);
		}
		else
		{
			guidedFilterSrc1Guidance3_(v[0],guidance,d[0],radius, eps);
			guidedFilterSrc1Guidance3_(v[1],guidance,d[1],radius, eps);
			guidedFilterSrc1Guidance3_(v[2],guidance,d[2],radius, eps);

		}
		merge(d,dest);
	}
	else if(src.channels()==3 && guidance.channels()==1)
	{
		vector<Mat> v;
		vector<Mat> d(3);
		split(src,v);
		if(sse)
		{
			guidedFilterSrc1Guidance1SSE_(v[0],guidance,d[0],radius, eps);
			guidedFilterSrc1Guidance1SSE_(v[1],guidance,d[1],radius, eps);
			guidedFilterSrc1Guidance1SSE_(v[2],guidance,d[2],radius, eps);
		}
		else
		{
			guidedFilterSrc1Guidance1_(v[0],guidance,d[0],radius, eps);
			guidedFilterSrc1Guidance1_(v[1],guidance,d[1],radius, eps);
			guidedFilterSrc1Guidance1_(v[2],guidance,d[2],radius, eps);
		}
		merge(d,dest);
	}
}


static void guidedFilterSrc1SSE_(const Mat& src, const Mat& dest,const int radius,const float eps)
{	
	
	if(src.channels()!=1)
	{
		cout<<"Please input gray scale image."<<endl;
		return;
	}
	Size ksize(2*radius+1,2*radius+1);
	Size imsize = src.size();
	//const int sseims = imsize.area()/4;
	const int sseims = 0;
	const int  nn = imsize.area() - sseims*4;
	const float e=eps*eps;

	Mat sf;src.convertTo(sf,CV_32F,1.0/255);
	Mat mSrc(imsize,CV_32F);//mean_p
	Mat x1(imsize,CV_32F),x2(imsize,CV_32F),x3(imsize,CV_32F);

	boxFilter2(sf,mSrc,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//mSrc*K
	multiplySSE_float(sf,x1);//sf*sf
	boxFilter2(x1,x3,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//m*sf*sf
	{
		float* s1=mSrc.ptr<float>(0);
		float* s2=x3.ptr<float>(0);
		float* s3=x1.ptr<float>(0);
		const __m128 ms4 = _mm_set1_ps(e);
		for(int i=sseims;i--;)
		{
			const __m128 ms1 = _mm_load_ps(s1);
			__m128 ms2 = _mm_mul_ps(ms1,ms1);
			__m128 ms3 = _mm_load_ps(s2);

			ms3 = _mm_sub_ps(ms3,ms2);
			ms2 = _mm_add_ps(ms3,ms4);
			ms3 = _mm_div_ps(ms3,ms2);
			_mm_store_ps(s2,ms3);
			ms3 = _mm_mul_ps(ms3,ms1);
			ms3 = _mm_sub_ps(ms3,ms1);
			_mm_store_ps(s3,ms3);

			s1+=4,s2+=4,s3+=4;
		}
		for(int i=0;i<nn;i++)
		{
			const float v = *s2 - (*s1 **s1); 
			*s2 = v / (v + e);
			*s3 = (*s2 * *s1) - *s1;
			s1++,s2++,s3++;
		}
		
	}
	boxFilter2(x3,x2,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x2*k
	boxFilter2(x1,x3,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x3*k
	{

		float* s1=x2.ptr<float>(0);
		float* s2=sf.ptr<float>(0);
		float* s3=x3.ptr<float>(0);
		const __m128 ms3 = _mm_set1_ps(255.f);
		for(int i=sseims;i--;)
		{
			__m128 ms1 = _mm_load_ps(s1);
			__m128 ms2 = _mm_load_ps(s2);
			ms1 = _mm_mul_ps(ms1,ms2);

			ms2 = _mm_load_ps(s3);
			ms1 = _mm_sub_ps(ms1,ms2);

			ms2 = _mm_mul_ps(ms1,ms3);
			_mm_store_ps(s1,ms2);

			s1+=4,s2+=4,s3+=4;
		}
		for(int i=0;i<nn;i++)
		{
			*s1 = 255.f*((*s1 * *s2) - *s3);
			s1++,s2++,s3++;
		}
	}
	x2.convertTo(dest,src.type());
}

static void guidedFilterSrc1_(const Mat& src, const Mat& dest, const int radius, const float eps)
{	
	if(src.channels()!=1)
	{
		cout<<"Please input gray scale image."<<endl;
		return;
	}
	Size ksize(2*radius+1,2*radius+1);
	Size imsize = src.size();
	const float e=eps*eps;

	Mat sf;src.convertTo(sf,CV_32F,1.0/255);
	Mat mSrc(imsize,CV_32F);//mean_p

	boxFilter(sf,mSrc,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//mSrc*K
	Mat x1(imsize,CV_32F),x2(imsize,CV_32F),x3(imsize,CV_32F);

	multiply(sf,sf,x1);//sf*sf
	boxFilter(x1,x3,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//m*sf*sf

	multiply(mSrc,mSrc,x1);//;msf*msf
	x3-=x1;//x3: m*sf*sf-msf*msf;
	x1 = x3+e;
	divide(x3,x1,x3);
	multiply(x3,mSrc,x1);
	x1-=mSrc;
	boxFilter(x3,x2,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x2*k
	boxFilter(x1,x3,CV_32F,ksize,Point(-1,-1),true,BORDER_TYPE);//x3*k
	multiply(x2,sf,x1);//x1*K
	x2 = x1-x3;//
	x2.convertTo(dest,src.type(),255);
}

void guidedFilter(const Mat& src,  Mat& dest, const int radius,const float eps)
{
	if(radius==0){src.copyTo(dest);return;}
	bool sse = checkHardwareSupport(CV_CPU_SSE2);
	if(radius == 0)
		src.copyTo(dest);

	if(src.channels()==1)
	{
		if(sse)
		{
			guidedFilterSrc1SSE_(src,dest,radius,eps);
		}
		else
		{
			guidedFilterSrc1_(src,dest,radius,eps);
		}
	}
	else if(src.channels()==3 )
	{
		vector<Mat> v;
		vector<Mat> d(3);
		split(src,v);
		if(sse)
		{
			guidedFilterSrc1Guidance3SSE_(v[0],src,d[0],radius, eps);
			guidedFilterSrc1Guidance3SSE_(v[1],src,d[1],radius, eps);
			guidedFilterSrc1Guidance3SSE_(v[2],src,d[2],radius, eps);
		}
		else
		{
			guidedFilterSrc1Guidance3_(v[0],src,d[0],radius, eps);
			guidedFilterSrc1Guidance3_(v[1],src,d[1],radius, eps);
			guidedFilterSrc1Guidance3_(v[2],src,d[2],radius, eps);

		}
		merge(d,dest);
	}
}

struct GuidedFilterInvoler
{
	const Mat *src2;
	const Mat *guidance2;
	Mat* dest2;
	int r;
	float eps;
	int MAXINDEX;
	bool isGuide;
	GuidedFilterInvoler(const Mat& src_,Mat& dest_, int r_,float eps_, int maxindex): 
	src2(&src_), dest2(&dest_), r(r_), eps(eps_),MAXINDEX(maxindex)
	{	
		isGuide = false;
	}

	GuidedFilterInvoler(const Mat& src_, const Mat& guidance_, Mat& dest_,int r_,float eps_, int maxindex): 
	src2(&src_), guidance2(&guidance_), dest2(&dest_), r(r_), eps(eps_),MAXINDEX(maxindex)
	{
		isGuide = true;
	}

	void operator() (int i) const
	{
		Mat src = (Mat)*src2;
		Mat dest = (Mat)*dest2;
		
		if(MAXINDEX==1)
		{
			Mat dst = dest;
			Mat s = src;
			
			if(!isGuide)
			{
				guidedFilter(s,dst,r,eps);
			}
			else
			{
				Mat guidance = (Mat)*guidance2;
				guidedFilter(s,guidance,dst,r,eps);
			}
			return ;
		}

		int rr = r;
		rr+=rr;

		int dh;
		int wss;
		int wst;
		if(i%2==0)
		{
			wst=0;
			wss = src.size().width/2 +rr;
			dh=src.size().width/2;
		}
		else
		{
			wst=src.size().width/2 -rr;
			wss = src.size().width-src.size().width/2 +rr;
			dh  = src.size().width-src.size().width/2;
		}
		int dv = src.size().height/(MAXINDEX/2);
		{
			int hst = dv*(i/2)-rr;
			int hss = dv+2*rr;
			if(MAXINDEX<=2)
			{
				hst=0;
				hss=src.size().height;
			}
			else
			{
				if(MAXINDEX<=4)
				{
					if(i==MAXINDEX-1 ||i==MAXINDEX-2)
					{
						hst = (dv*(i/2))-rr;
						hss = src.size().height - (dv*(i/2))+rr;
					}
					else
					{
						hst = 0;
						hss = dv+rr;
					}
				}
				else
				{
					if(i==MAXINDEX-1 ||i==MAXINDEX-2)
					{
						hst = (dv*(i/2))-rr;
						hss = src.size().height - (dv*(i/2))+rr;
					}
					else if(i==0 || i==1)
					{
						hst = 0;
						hss = dv+rr;
					}
				}
			}
			Mat ds = Mat::zeros(Size(wss,hss),dest.type());
			Mat s = src(Rect(wst,hst,wss,hss)).clone();
			
			if(!isGuide)
			{
				guidedFilter(s,ds,r,eps);
			}
			else
			{
				Mat guidance = (Mat)*guidance2;
				Mat g = guidance(Rect(wst,hst,wss,hss)).clone();
				guidedFilter(s,g,ds,r,eps);
			}
		
			//output data copy
			if(i==0)
			{
				ds(Rect(0,0,dh,dv)).copyTo(dest(Rect(0,0,dh,dv)));
			}
			else if(i==1)
			{
				ds(Rect(rr,0,dh,dv)).copyTo(dest(Rect(wst+rr,0,dh,dv)));
			}
			else if(i==MAXINDEX-2)
			{
				int vvv = src.rows -  (i/2)*dv;

				ds(Rect(0,rr,dh,vvv)).copyTo(dest(Rect(0,hst+rr,dh,vvv)));
			}
			else if(i==MAXINDEX-1)
			{
				int vvv = src.rows -  (i/2)*dv;
				ds(Rect(rr,rr,dh,vvv)).copyTo(dest(Rect(wst+rr,hst+rr,dh,vvv)));
			}
			else if(i%2==0)
			{
				ds(Rect(0,rr,dh,dv)).copyTo(dest(Rect(0,hst+rr,dh,dv)));
			}
			else
			{
				ds(Rect(rr,rr,dh,dv)).copyTo(dest(Rect(wst+rr,hst+rr,dh,dv)));
			}
			//imshow("g",dest);waitKey();
		}
	}
};

	void guidedFilterTBB(const Mat& src, Mat& dest, int r,float eps, const int threadmax)
	{
		dest.create(src.size(),src.type());

		vector<int> index(max(threadmax,1));
		int* idx = &index[0];
		for(int i=0;i<threadmax;i++)index[i]=i;
		
		GuidedFilterInvoler body(src,dest,r,eps,threadmax);
		parallel_do(idx,idx+threadmax,body);
	}

	void guidedFilterTBB(const Mat& src, const Mat& guidance, Mat& dest, int r,float eps, const int threadmax)
	{
		dest.create(src.size(),src.type());

		vector<int> index(max(threadmax,1));
		int* idx = &index[0];
		for(int i=0;i<threadmax;i++)index[i]=i;
		
		GuidedFilterInvoler body(src,guidance,dest,r,eps,threadmax);
		parallel_do(idx,idx+threadmax,body);
	}




/*
void guidedFilter_matlabconverted(Mat& src, Mat& joint,Mat& dest,const int radius,const double eps)
{	
//direct

if(src.channels()!=1 && joint.channels()!=1)
{
cout<<"Please input gray scale image."<<endl;
return;
}

Size ksize(2*radius+1,2*radius+1);

Mat mJoint;//mean_I
Mat mSrc;//mean_p
boxFilter(joint,mJoint,CV_32F,ksize,Point(-1,-1),true);
boxFilter(src,mSrc,CV_32F,ksize,Point(-1,-1),true);

Mat SxJ;//I.*p
multiply(joint,src,SxJ,1.0,CV_32F);
Mat mSxJ;//mean_Ip
boxFilter(SxJ,mSxJ,CV_32F,ksize,Point(-1,-1),true);

Mat mSxmJ;//mean_I.*mean_p
multiply(mJoint,mSrc,mSxmJ,1.0,CV_32F);
Mat covSxJ =mSxJ - mSxmJ;//cov_Ip

Mat joint2;
Mat mJointpow;
Mat joint32;
joint.convertTo(joint32,CV_32F);
cv::pow(joint32,2.0,joint2);
cv::pow(mJoint,2.0,mJointpow);

Mat mJoint2;
boxFilter(joint2,mJoint2,CV_32F,ksize,Point(-1,-1),true);//convert pow2&boxf32�t�B���^��

Mat vJoint = mJoint2 - mJointpow;

const double e=eps*eps;
vJoint = vJoint+e;

Mat a;
divide(covSxJ,vJoint,a);

Mat amJoint;
multiply(a,mJoint,amJoint,1.0,CV_32F);
Mat b = mSrc - amJoint;

Mat ma;
Mat mb;
boxFilter(a,ma,CV_32F,ksize,Point(-1,-1),true);
boxFilter(b,mb,CV_32F,ksize,Point(-1,-1),true);

Mat maJoint;
multiply(ma,joint32,maJoint,1.0,CV_32F);
Mat temp = maJoint+mb;

temp.convertTo(dest,dest.type());
}
*/
/*
void guidedFilterSrcGrayGuidanceColorNonop(Mat& src, Mat& guidance, Mat& dest, const int radius,const float eps)
{
if(src.channels()!=1 && guidance.channels()!=3)
{
cout<<"Please input gray scale image as src, and color image as guidance."<<endl;
return;
}
vector<Mat> I;
split(guidance,I);
const Size ksize(2*radius+1,2*radius+1);
const Point PT(-1,-1);
const double e=eps*eps;
const int size = src.size().area();

Mat mean_I_r;
Mat mean_I_g;
Mat mean_I_b;

Mat mean_p;

//cout<<"mean computation"<<endl;
boxFilter(I[0],mean_I_r,CV_32F,ksize,PT,true);
boxFilter(I[1],mean_I_g,CV_32F,ksize,PT,true);
boxFilter(I[2],mean_I_b,CV_32F,ksize,PT,true);

boxFilter(src,mean_p,CV_32F,ksize,PT,true);

Mat mean_Ip_r;
Mat mean_Ip_g;
Mat mean_Ip_b;
multiply(I[0],src,mean_Ip_r,1.0,CV_32F);//Ir*p
boxFilter(mean_Ip_r,mean_Ip_r,CV_32F,ksize,PT,true);
multiply(I[1],src,mean_Ip_g,1.0,CV_32F);//Ig*p
boxFilter(mean_Ip_g,mean_Ip_g,CV_32F,ksize,PT,true);
multiply(I[2],src,mean_Ip_b,1.0,CV_32F);//Ib*p
boxFilter(mean_Ip_b,mean_Ip_b,CV_32F,ksize,PT,true);

//cout<<"covariance computation"<<endl;
Mat cov_Ip_r;
Mat cov_Ip_g;
Mat cov_Ip_b;
multiply(mean_I_r,mean_p,cov_Ip_r,-1.0,CV_32F);
cov_Ip_r += mean_Ip_r;
multiply(mean_I_g,mean_p,cov_Ip_g,-1.0,CV_32F);
cov_Ip_g += mean_Ip_g;
multiply(mean_I_b,mean_p,cov_Ip_b,-1.0,CV_32F);
cov_Ip_b += mean_Ip_b;



//cout<<"variance computation"<<endl;
Mat temp;
Mat var_I_rr;
multiply(I[0],I[0],temp,1.0,CV_32F);
boxFilter(temp,var_I_rr,CV_32F,ksize,PT,true);
multiply(mean_I_r,mean_I_r,temp,1.0,CV_32F);
var_I_rr-=temp;

var_I_rr+=eps;

Mat var_I_rg;
multiply(I[0],I[1],temp,1.0,CV_32F);
boxFilter(temp,var_I_rg,CV_32F,ksize,PT,true);
multiply(mean_I_r,mean_I_g,temp,1.0,CV_32F);
var_I_rg-=temp;
Mat var_I_rb;
multiply(I[0],I[2],temp,1.0,CV_32F);
boxFilter(temp,var_I_rb,CV_32F,ksize,PT,true);
multiply(mean_I_r,mean_I_b,temp,1.0,CV_32F);
var_I_rb-=temp;
Mat var_I_gg;
multiply(I[1],I[1],temp,1.0,CV_32F);
boxFilter(temp,var_I_gg,CV_32F,ksize,PT,true);
multiply(mean_I_g,mean_I_g,temp,1.0,CV_32F);
var_I_gg-=temp;

var_I_gg+=eps;

Mat var_I_gb;
multiply(I[1],I[2],temp,1.0,CV_32F);
boxFilter(temp,var_I_gb,CV_32F,ksize,PT,true);
multiply(mean_I_g,mean_I_b,temp,1.0,CV_32F);
var_I_gb-=temp;
Mat var_I_bb;
multiply(I[2],I[2],temp,1.0,CV_32F);
boxFilter(temp,var_I_bb,CV_32F,ksize,PT,true);
multiply(mean_I_b,mean_I_b,temp,1.0,CV_32F);
var_I_bb-=temp;

var_I_bb+=eps;


float* rr = var_I_rr.ptr<float>(0);
float* rg = var_I_rg.ptr<float>(0);
float* rb = var_I_rb.ptr<float>(0);
float* gg = var_I_gg.ptr<float>(0);
float* gb = var_I_gb.ptr<float>(0);
float* bb = var_I_bb.ptr<float>(0);

Mat a_r = Mat::zeros(src.size(),CV_32F);
Mat a_g = Mat::zeros(src.size(),CV_32F);
Mat a_b = Mat::zeros(src.size(),CV_32F);
float* ar = a_r.ptr<float>(0);
float* ag = a_g.ptr<float>(0);
float* ab = a_b.ptr<float>(0);
float* covr = cov_Ip_r.ptr<float>(0);
float* covg = cov_Ip_g.ptr<float>(0);
float* covb = cov_Ip_b.ptr<float>(0);

for(int i=0;i<size;i++)
{
//�t�s�񖳂���3ms

Matx33f sigmaEps
(
rr[i],rg[i],rb[i],
rg[i],gg[i],gb[i],
rb[i],gb[i],bb[i]
);

//Matx33f inv = sigmaEps.inv(cv::DECOMP_CHOLESKY);
Matx33f inv = sigmaEps.inv(cv::DECOMP_LU);
//Matx33f inv2 = sigmaEps.inv(cv::DECOMP_LU);

ar[i]= covr[i]*inv(0,0) + covg[i]*inv(1,0) + covb[i]*inv(2,0);
ag[i]= covr[i]*inv(0,1) + covg[i]*inv(1,1) + covb[i]*inv(2,1);
ab[i]= covr[i]*inv(0,2) + covg[i]*inv(1,2) + covb[i]*inv(2,2);
}

multiply(a_r,mean_I_r,mean_I_r,-1.0,CV_32F);//break mean_I_r;
multiply(a_g,mean_I_g,mean_I_g,-1.0,CV_32F);//break mean_I_g;
multiply(a_b,mean_I_b,mean_I_b,-1.0,CV_32F);//break mean_I_b;
Mat b = mean_p + mean_I_r+ mean_I_g + mean_I_b;

boxFilter(a_r,a_r,CV_32F,ksize,PT,true);//break a_r
boxFilter(a_g,a_g,CV_32F,ksize,PT,true);//break a_g
boxFilter(a_b,a_b,CV_32F,ksize,PT,true);//break a_b

boxFilter(b,temp,CV_32F,ksize,PT,true);
multiply(a_r,I[0],a_r,1.0,CV_32F);
multiply(a_g,I[1],a_g,1.0,CV_32F);
multiply(a_b,I[2],a_b,1.0,CV_32F);
temp = temp + a_r + a_g + a_b;

temp.convertTo(dest,src.type());
}
*/