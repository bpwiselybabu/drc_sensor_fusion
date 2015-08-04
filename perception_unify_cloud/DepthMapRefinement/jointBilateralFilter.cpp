#include "filter.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//baseline 
void jointBilateralFilterBase_32f( const Mat& src,const Mat& joint, Mat& dst, int d,
	double sigma_color, double sigma_space,int borderType)
{

	Mat aaa;src.convertTo(aaa,CV_8U,255.0/1500.0);imshow("test1",aaa);

	if(d==0){src.copyTo(dst);return;}
	Size size = src.size();
	if(dst.empty())dst=Mat::zeros(src.size(),src.type());

	CV_Assert( (src.type() == CV_32FC1 || src.type() == CV_32FC3) &&
		src.type() == dst.type() && src.size() == dst.size());

	if( sigma_color <= 0.0 )
		sigma_color = 1.0;
	if( sigma_space <= 0.0 )
		sigma_space = 1.0;

	double gauss_color_coeff = -0.5/(sigma_color*sigma_color);
	double gauss_space_coeff = -0.5/(sigma_space*sigma_space);

	const int cn = src.channels();
	const int cnj = joint.channels();

	int radius;
	/*if( d <= 0 )
		radius = cvRound(sigma_space*1.5);
	else*/
		radius = d/2;
	//radius = MAX(radius, 1);
	d = radius*2 + 1;

	Mat jim;
	Mat sim;
	
	copyMakeBorder( joint, jim, radius, radius, radius, radius, borderType );
	copyMakeBorder( src, sim, radius, radius, radius, radius, borderType );

	vector<float> _color_weight(cnj*256);
	vector<float> _space_weight(d*d);
	float* color_weight = &_color_weight[0];
	float* space_weight = &_space_weight[0];

	vector<int> _space_ofs_src(d*d);
	vector<int> _space_ofs_jnt(d*d);
	int* space_ofs_src = &_space_ofs_src[0];
	int* space_ofs_jnt = &_space_ofs_jnt[0];
	
	
	// initialize color-related bilateral filter coefficients
	for(int i = 0; i < 256*cnj; i++ )
		color_weight[i] = (float)std::exp(i*i*gauss_color_coeff);

	int maxk=0;
	// initialize space-related bilateral filter coefficients
	for(int i = -radius; i <= radius; i++ )
	{
		for(int j = -radius; j <= radius; j++ )
		{
			double r = std::sqrt((double)i*i + (double)j*j);
			if( r > radius )
				continue;
			space_weight[maxk] = (float)std::exp(r*r*gauss_space_coeff);
			space_ofs_jnt[maxk] = (int)(i*jim.cols*cnj + j*cnj);
			space_ofs_src[maxk++] = (int)(i*sim.cols*cn + j*cn);
		}
	}

	if(!src.isContinuous())cout<<"NG\n";
	if(!joint.isContinuous())cout<<"NG\n";
	if(!dst.isContinuous())cout<<"NG\n";
	/*cout<<format("%d %d\n",size.width,size.height);
	cout<<format("%d %d\n",src.cols,src.rows);
	cout<<format("%d %d\n",joint.cols,joint.rows);
	cout<<format("%d %d\n",dst.cols,dst.rows);
	cout<<format("D %d maxk %d \n",d,maxk);*/
	for(int i = 0; i < size.height; i++ )
	{
		const float* jptr = jim.ptr<float>(i+radius)+ radius*cnj;
		const float* sptr = sim.ptr<float>(i+radius)+ radius*cn;
		float* dptr = dst.ptr<float>(i);

		if( cn == 1 && cnj==1)
		{
			for(int j = 0; j < size.width; j++ )
			{
				float sum = 0.f, wsum = 0.f;
				float val0 = jptr[j];
				for(int k = 0; k < maxk; k++ )
				{
					float val = jptr[j + space_ofs_src[k]];
					float vals = sptr[j + space_ofs_src[k]];
					float w = space_weight[k]*color_weight[cvRound(std::abs(val - val0))];
					sum += vals*w;
					wsum += w;
				}
				dptr[j] = sum/wsum;
			}
		}
		else if(cn == 3 && cnj==3)
		{
			for(int j = 0; j < size.width*3; j += 3 )
			{
				float sum_b = 0.f, sum_g = 0.f, sum_r = 0.f, wsum = 0.f;
				const float b0j = jptr[j], g0j = jptr[j+1], r0j = jptr[j+2];

				for(int k = 0; k < maxk; k++ )
				{
					const float* jptr_k = jptr + j + space_ofs_src[k];
					const float bj = jptr_k[0], gj = jptr_k[1], rj = jptr_k[2];
					const float* sptr_k = sptr + j + space_ofs_src[k];
					const float b = sptr_k[0], g = sptr_k[1], r = sptr_k[2];
					
					float w = space_weight[k]
					*color_weight[cvRound(std::abs(bj - b0j) +std::abs(gj - g0j) + std::abs(rj - r0j))];
					sum_b += b*w; 
					sum_g += g*w;
					sum_r += r*w;
					wsum += w;
				}
				dptr[j  ] = sum_b/wsum;
				dptr[j+1] = sum_g/wsum;
				dptr[j+2] = sum_r/wsum;
			}
		}
		else if(cn == 1 && cnj==3)
		{
			for(int j = 0, l=0; j < size.width*3; j += 3,l++ )
			{
				float sum_b = 0,wsum = 0;
				float b0 = jptr[j], g0 = jptr[j+1], r0 = jptr[j+2];
				for(int k = 0; k < maxk; k++ )
				{
					const float* sptr_k = jptr + j + space_ofs_jnt[k];
					float val = *(sptr + l + space_ofs_src[k]);
					float b = sptr_k[0], g = sptr_k[1], r = sptr_k[2];
					float w = space_weight[k]*color_weight[cvRound(std::abs(b - b0) +
						std::abs(g - g0) + std::abs(r - r0))];
					sum_b += val*w; 
					wsum += w;
				}
				wsum = 1.f/wsum;
				dptr[l] = sum_b*wsum;
			}
		}
		else if(cn == 3 && cnj==1)
		{
			for(int j = 0,l=0; j < size.width*3; j+=3,l++ )
			{
				float sum_b = 0.f, sum_g = 0.f, sum_r = 0.f, wsum = 0.f;
				const float val0 = jptr[l];
				for(int k = 0; k < maxk; k++ )
				{
					float val = jptr[l + space_ofs_jnt[k]];
					const float* sptr_k = sptr + j + space_ofs_src[k];

					float w = space_weight[k]*color_weight[cvRound(std::abs(val - val0))];
					sum_b += sptr_k[0]*w; sum_g += sptr_k[1]*w; sum_r += sptr_k[2]*w;
					wsum += w;
				}
				// overflow is not possible here => there is no need to use CV_CAST_8U
				wsum = 1.f/wsum;
				dptr[j  ] = sum_b*wsum;
				dptr[j+1] = sum_g*wsum;
				dptr[j+2] = sum_r*wsum;
			}
		}
	}
}

void jointBilateralFilterBase_8u( const Mat& src,const Mat& joint, Mat& dst, int d,
	double sigma_color, double sigma_space,int borderType)
{
	if(d==0){src.copyTo(dst);return;}
	Size size = src.size();
	if(dst.empty())dst=Mat::zeros(src.size(),src.type());
	//CV_Assert( (src.type() == CV_8UC1 || src.type() == CV_8UC3) &&
	//	src.type() == dst.type() && src.size() == dst.size() &&
	//	src.data != dst.data );

	if( sigma_color <= 0.0 )
		sigma_color = 1.0;
	if( sigma_space <= 0.0 )
		sigma_space = 1.0;
	double gauss_color_coeff = -0.5/(sigma_color*sigma_color);
	double gauss_space_coeff = -0.5/(sigma_space*sigma_space);

	if(joint.empty())src.copyTo(joint);
	const int cn = src.channels();
	const int cnj = joint.channels();

	int radius;
	if( d <= 0 )
		radius = cvRound(sigma_space*1.5);
	else
		radius = d/2;
	radius = MAX(radius, 1);
	d = radius*2 + 1;

	Mat jim;
	Mat sim;
	copyMakeBorder( joint, jim, radius, radius, radius, radius, borderType );
	copyMakeBorder( src, sim, radius, radius, radius, radius, borderType );

	vector<float> _color_weight(cnj*256);
	vector<float> _space_weight(d*d);
	vector<int> _space_ofs_jnt(d*d);
	vector<int> _space_ofs_src(d*d);
	float* color_weight = &_color_weight[0];
	float* space_weight = &_space_weight[0];
	int* space_ofs_jnt = &_space_ofs_jnt[0];
	int* space_ofs_src = &_space_ofs_src[0];

	// initialize color-related bilateral filter coefficients
	for(int i = 0; i < 256*cnj; i++ )
		color_weight[i] = (float)std::exp(i*i*gauss_color_coeff);

	int maxk=0;
	// initialize space-related bilateral filter coefficients
	for(int i = -radius; i <= radius; i++ )
	{
		for(int j = -radius; j <= radius; j++ )
		{
			double r = std::sqrt((double)i*i + (double)j*j);
			if( r > radius )
				continue;
			space_weight[maxk] = (float)std::exp(r*r*gauss_space_coeff);
			space_ofs_jnt[maxk] = (int)(i*jim.step + j*cnj);
			space_ofs_src[maxk++] = (int)(i*sim.step + j*cn);
		}
	}

	//#pragma omp parallel for
	for(int i = 0; i < size.height; i++ )
	{
		const uchar* jptr = jim.data + (i+radius)*jim.step + radius*cnj;
		const uchar* sptr = sim.data + (i+radius)*sim.step + radius*cn;
		uchar* dptr = dst.data + i*dst.step;

		if( cn == 1 && cnj==1)
		{
			for(int j = 0; j < size.width; j++ )
			{
				float sum = 0, wsum = 0;
				int val0 = jptr[j];
				for(int k = 0; k < maxk; k++ )
				{
					int val = jptr[j + space_ofs_src[k]];
					int val2 = sptr[j + space_ofs_src[k]];
					float w = space_weight[k]*color_weight[std::abs(val - val0)];
					sum += val2*w;
					wsum += w;
				}
				// overflow is not possible here => there is no need to use CV_CAST_8U
				dptr[j] = (uchar)cvRound(sum/wsum);
			}
		}
		else if(cn == 3 &&cnj==3)
		{
			for(int j = 0; j < size.width*3; j += 3 )
			{
				float sum_b = 0, sum_g = 0, sum_r = 0, wsum = 0;
				int b0 = jptr[j], g0 = jptr[j+1], r0 = jptr[j+2];
				for(int k = 0; k < maxk; k++ )
				{
					const uchar* sptr_k = jptr + j + space_ofs_src[k];
					const uchar* sptr_k2 = sptr + j + space_ofs_src[k];
					int b = sptr_k[0], g = sptr_k[1], r = sptr_k[2];
					float w = space_weight[k]*color_weight[std::abs(b - b0) +
						std::abs(g - g0) + std::abs(r - r0)];
					sum_b += sptr_k2[0]*w; sum_g += sptr_k2[1]*w; sum_r += sptr_k2[2]*w;
					wsum += w;
				}
				wsum = 1.f/wsum;
				b0 = cvRound(sum_b*wsum);
				g0 = cvRound(sum_g*wsum);
				r0 = cvRound(sum_r*wsum);
				dptr[j] = (uchar)b0; dptr[j+1] = (uchar)g0; dptr[j+2] = (uchar)r0;
			}
		}
		else if(cn == 1 && cnj==3)
		{
			for(int j = 0, l=0; l < size.width; j += 3,l++ )
			{
				float sum_b = 0.f,wsum = 0.f;
				int b0 = jptr[j], g0 = jptr[j+1], r0 = jptr[j+2];
				for(int k = 0; k < maxk; k++ )
				{
					const uchar* sptr_k = jptr + j + space_ofs_jnt[k];
					int val = *(sptr + l + space_ofs_src[k]);
					int b = sptr_k[0], g = sptr_k[1], r = sptr_k[2];
					float w = space_weight[k]*color_weight[std::abs(b - b0) +
						std::abs(g - g0) + std::abs(r - r0)];
					sum_b += val*w; 
					wsum += w;
				}
				wsum = 1.f/wsum;
				b0 = cvRound(sum_b*wsum);
				dptr[l] = (uchar)b0;
			}
		}
		else if(cn == 3 && cnj==1)
		{
			for(int j = 0,l=0; l < size.width; j+=3,l++ )
			{
				float sum_b = 0.f, sum_g = 0.f, sum_r = 0.f, wsum = 0.f;
				const int val0 = jptr[l];
				for(int k = 0; k < maxk; k++ )
				{
					int val = jptr[l + space_ofs_jnt[k]];
					const uchar* sptr_k = sptr + j + space_ofs_src[k];


					float w = space_weight[k]*color_weight[std::abs(val - val0)];
					sum_b += sptr_k[0]*w; 
					sum_g += sptr_k[1]*w; 
					sum_r += sptr_k[2]*w;
					wsum += w;
				}
				// overflow is not possible here => there is no need to use CV_CAST_8U
				wsum = 1.f/wsum;
				dptr[j] = (uchar)cvRound(sum_b*wsum); 
				dptr[j+1] = (uchar)cvRound(sum_g*wsum); 
				dptr[j+2] = (uchar)cvRound(sum_r*wsum);
			}
		}
	}
}

void jointBilateralFilterBase( const Mat& src, const Mat& joint, Mat& dst, int d,
	double sigma_color, double sigma_space,int borderType)
{
	if(src.type()==CV_MAKE_TYPE(CV_8U,src.channels()))
	{
		jointBilateralFilterBase_8u(src,joint,dst,d,sigma_color,sigma_space,borderType);
	}
	else if(src.type()==CV_MAKE_TYPE(CV_32F,src.channels()))
	{
		jointBilateralFilterBase_32f(src,joint,dst,d,sigma_color,sigma_space,borderType);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class WeightedJointBilateralFilter_32f_InvokerSSE4 : public cv::ParallelLoopBody
{
public:
	WeightedJointBilateralFilter_32f_InvokerSSE4(Mat& _dest, const Mat& _temp, const Mat& _weightMap,const Mat& _guide, int _radiusH, int _radiusV, int _maxk,
		int* _space_ofs, int* _space_w_ofs,int* _space_guide_ofs, float *_space_weight, float *_color_weight) :
	temp(&_temp), weightMap(&_weightMap),dest(&_dest), guide(&_guide), radiusH(_radiusH), radiusV(_radiusV),
		maxk(_maxk), space_ofs(_space_ofs), space_w_ofs(_space_w_ofs),space_guide_ofs(_space_guide_ofs), space_weight(_space_weight), color_weight(_color_weight)
	{
	}

	virtual void operator() (const Range& range) const
	{
		int i, j, cn = dest->channels(), k;
		int cng = (guide->rows-2*radiusV) / dest->rows;
		Size size = dest->size();

		static int CV_DECL_ALIGNED(16) v32f_absmask[] = { 0x7fffffff, 0x7fffffff, 0x7fffffff, 0x7fffffff };

#if CV_SSE4_1
		bool haveSSE4 = checkHardwareSupport(CV_CPU_SSE4_1);
#endif
		if( cn == 1 && cng ==1)
		{
			int CV_DECL_ALIGNED(16) buf[4];

			float* sptr = (float*)temp->ptr<float>(range.start+radiusV) + 4 * (radiusH/4 + 1);
			float* gptr = (float*)guide->ptr<float>(range.start+radiusV) + 4 * (radiusH/4 + 1);
			float* wptr = (float*)weightMap->ptr<float>(range.start+radiusV) + 4 * (radiusH/4 + 1);//wmap!

			float* dptr = dest->ptr<float>(range.start);

			const int sstep = temp->cols;
			const int gstep = guide->cols;
			const int wstep = weightMap->cols;//wmap!

			const int dstep = dest->cols;

			for(i = range.start; i != range.end; i++,dptr+=dstep,sptr+=sstep,gptr+=gstep,wptr+=wstep)//wmap!
			{
				j=0;
#if CV_SSE4_1
				if( haveSSE4 )
				{
					for(; j < size.width; j+=4)//4 pixel unit
					{
						int* ofs = &space_ofs[0];
						int* gofs = &space_guide_ofs[0];
						int* wofs = &space_w_ofs[0];//wmap!

						float* spw = space_weight;

						const float* sptrj = sptr+j;
						const float* gptrj = gptr+j;
						const float* wptrj = wptr+j;//wmap!

						const __m128 sval = _mm_load_ps((gptrj));
						
						__m128 wval1 = _mm_setzero_ps();
						__m128 tval1 = _mm_setzero_ps();


						for(k = 0;  k <= maxk; k ++, ofs++,gofs++,spw++,wofs++)//wmap!
						{
							__m128 sref = _mm_loadu_ps((gptrj+*gofs));
							_mm_store_si128((__m128i*)buf,_mm_cvtps_epi32(_mm_and_ps(_mm_sub_ps(sval,sref), *(const __m128*)v32f_absmask)));

							__m128 vref = _mm_loadu_ps((sptrj+*ofs));
							const __m128 _sw = _mm_set1_ps(*spw);//space weight
							__m128 _w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[3]],color_weight[buf[2]],color_weight[buf[1]],color_weight[buf[0]]));//��������̐�Βl����exp��\��LUT�ɓ��Ă�������W�X�^�ɃX�g�A�i�F�d�݁j
							_w = _mm_mul_ps(_w,_mm_loadu_ps(wptrj+wofs[0]));//wmap!

							vref = _mm_mul_ps(_w, vref);//�l�Əd�ݑS�̂Ƃ̐�
							tval1 = _mm_add_ps(tval1,vref);
							wval1 = _mm_add_ps(wval1,_w);
						}
						tval1 = _mm_div_ps(tval1,wval1);
						_mm_stream_ps((dptr+j), tval1);
					}
				}
#endif
				for(; j < size.width; j++)
				{
					const float val0 = gptr[j];
					float sum=0.0f;
					float wsum=0.0f;
					for(k=0 ; k < maxk; k++ )
					{
						float gval = gptr[j + space_guide_ofs[k]];
						float val = sptr[j + space_ofs[k]];
						float w = space_weight[k]*color_weight[cvRound(std::abs(gval - val0))];
						sum += val*w;
						wsum += w;
					}
					dptr[j] = sum/wsum;
				}
			}
		}
		else if(cn == 1 && cng == 3)
		{
			assert( cng == 3 );//color
			int CV_DECL_ALIGNED(16) buf[4];

			float* sptr = (float*)temp->ptr<float>(range.start+radiusV) + 4 * (radiusH/4 + 1);
			float* gptrr = (float*)guide->ptr<float>(3*radiusV+3*range.start  ) + 4 * (radiusH/4 + 1);
			float* gptrg = (float*)guide->ptr<float>(3*radiusV+3*range.start+1) + 4 * (radiusH/4 + 1);
			float* gptrb = (float*)guide->ptr<float>(3*radiusV+3*range.start+2) + 4 * (radiusH/4 + 1);
			float* wptr = (float*)weightMap->ptr<float>(range.start+radiusV) + 4 * (radiusH/4 + 1);//wmap!

			float* dptr = dest->ptr<float>(range.start);

			const int sstep = temp->cols;
			const int gstep = 3*guide->cols;
			const int wstep = weightMap->cols;//wmap

			const int dstep = dest->cols;

			for(i = range.start; i != range.end; i++,gptrr+=gstep,gptrg+=gstep,gptrb+=gstep, sptr+=sstep, dptr+=dstep ,wptr+=wstep)//wmap!
			{	
				j=0;
#if CV_SSE4_1
				if( haveSSE4 )
				{
					for(; j < size.width; j+=4)//4 pixel unit
					{
						int* ofs = &space_ofs[0];
						int* gofs = &space_guide_ofs[0];
						int* wofs = &space_w_ofs[0];//wmap

						float* spw = space_weight;

						const float* sptrj = sptr+j;
						const float* gptrrj = gptrr+j;
						const float* gptrgj = gptrg+j;
						const float* gptrbj = gptrb+j;
						const float* wptrj = wptr+j;//wmap

						const __m128 bval = _mm_load_ps((gptrbj));
						const __m128 gval = _mm_load_ps((gptrgj));
						const __m128 rval = _mm_load_ps((gptrrj));

						__m128 wval1 = _mm_setzero_ps();
						__m128 tval1 = _mm_setzero_ps();
						for(k = 0;  k <= maxk; k ++, ofs++,gofs++,spw++,wofs++)//wmap
						{
							const __m128 bref = _mm_loadu_ps((gptrbj+*gofs));
							const __m128 gref = _mm_loadu_ps((gptrgj+*gofs));
							const __m128 rref = _mm_loadu_ps((gptrrj+*gofs));

							_mm_store_si128((__m128i*)buf,
								_mm_cvtps_epi32(
								_mm_add_ps(
								_mm_add_ps(
								_mm_and_ps(_mm_sub_ps(rval,rref), *(const __m128*)v32f_absmask),
								_mm_and_ps(_mm_sub_ps(gval,gref), *(const __m128*)v32f_absmask)),
								_mm_and_ps(_mm_sub_ps(bval,bref), *(const __m128*)v32f_absmask)
								)
								));
							__m128 vref = _mm_loadu_ps((sptrj+*ofs));
							const __m128 _sw = _mm_set1_ps(*spw);//space weight
							__m128 _w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[3]],color_weight[buf[2]],color_weight[buf[1]],color_weight[buf[0]]));//load exp val from lut
							_w = _mm_mul_ps(_w,_mm_loadu_ps(wptrj+wofs[0]));//wmap

							vref = _mm_mul_ps(_w, vref);//mul weight and val
							tval1 = _mm_add_ps(tval1,vref);
							wval1 = _mm_add_ps(wval1,_w);
						}
						tval1 = _mm_div_ps(tval1,wval1);
						_mm_stream_ps((dptr+j), tval1);
					}
				}
#endif
				for(; j < size.width; j++)
				{
					const float* sptrj = sptr+j;
					const float* gptrrj = gptrr+j;
					const float* gptrgj = gptrg+j;
					const float* gptrbj = gptrb+j;

					const float r0 = gptrrj[0];
					const float g0 = gptrgj[0];
					const float b0 = gptrbj[0];

					float sum=0.0f;
					float wsum=0.0f;
					for(k=0 ; k < maxk; k++ )
					{
						const float r = gptrrj[space_guide_ofs[k]], g = gptrgj[space_guide_ofs[k]], b = gptrbj[space_guide_ofs[k]];
						float w = space_weight[k]*color_weight[cvRound(std::abs(b - b0) +std::abs(g - g0) + std::abs(r - r0))];
						sum += sptrj[space_ofs[k]]*w;
						wsum += w;
					}
					dptr[j] = sum/wsum;
				}
			}
		}
		else if(cn == 3 && cng == 3)
		{
			assert( cng == 3 );//color
			int CV_DECL_ALIGNED(16) buf[4];

			float* sptrr =  (float*)temp->ptr<float>(3*radiusV+3*range.start  ) + 4 * (radiusH/4 + 1);
			float* sptrg =  (float*)temp->ptr<float>(3*radiusV+3*range.start+1) + 4 * (radiusH/4 + 1);
			float* sptrb =  (float*)temp->ptr<float>(3*radiusV+3*range.start+2) + 4 * (radiusH/4 + 1);
			float* gptrr = (float*)guide->ptr<float>(3*radiusV+3*range.start  ) + 4 * (radiusH/4 + 1);
			float* gptrg = (float*)guide->ptr<float>(3*radiusV+3*range.start+1) + 4 * (radiusH/4 + 1);
			float* gptrb = (float*)guide->ptr<float>(3*radiusV+3*range.start+2) + 4 * (radiusH/4 + 1);
			float* wptr = (float*)weightMap->ptr<float>(range.start+radiusV) + 4 * (radiusH/4 + 1);//wmap

			float* dptr = dest->ptr<float>(range.start);

			const int sstep = 3*temp->cols;
			const int gstep = 3*guide->cols;
			const int wstep = weightMap->cols;//wmap

			const int dstep = 3*dest->cols;

			for(i = range.start; i != range.end; i++,gptrr+=gstep,gptrg+=gstep,gptrb+=gstep, sptrr+=sstep,sptrg+=sstep,sptrb+=sstep, dptr+=dstep ,wptr+=wstep)//wmap!
			{	
				j=0;
#if CV_SSE4_1
				if( haveSSE4 )
				{
					for(; j < size.width; j+=4)//4 pixel unit
					{
						int* ofs = &space_ofs[0];
						int* gofs = &space_guide_ofs[0];
						int* wofs = &space_w_ofs[0];//wmap!

						float* spw = space_weight;

						const float* sptrrj = sptrr+j;
						const float* sptrgj = sptrg+j;
						const float* sptrbj = sptrb+j;
						const float* gptrrj = gptrr+j;
						const float* gptrgj = gptrg+j;
						const float* gptrbj = gptrb+j;
						const float* wptrj = wptr+j;//wmap!

						const __m128 bval = _mm_load_ps((gptrbj));
						const __m128 gval = _mm_load_ps((gptrgj));
						const __m128 rval = _mm_load_ps((gptrrj));

						__m128 wval1 = _mm_setzero_ps();
						__m128 rval1 = _mm_setzero_ps();
						__m128 gval1 = _mm_setzero_ps();
						__m128 bval1 = _mm_setzero_ps();

						for(k = 0;  k <= maxk; k ++, ofs++,gofs++,spw++,wofs++)//wmap
						{
							__m128 bref = _mm_loadu_ps((gptrbj+*gofs));
							__m128 gref = _mm_loadu_ps((gptrgj+*gofs));
							__m128 rref = _mm_loadu_ps((gptrrj+*gofs));

							_mm_store_si128((__m128i*)buf,
								_mm_cvtps_epi32(
								_mm_add_ps(
								_mm_add_ps(
								_mm_and_ps(_mm_sub_ps(rval,rref), *(const __m128*)v32f_absmask),
								_mm_and_ps(_mm_sub_ps(gval,gref), *(const __m128*)v32f_absmask)),
								_mm_and_ps(_mm_sub_ps(bval,bref), *(const __m128*)v32f_absmask)
								)
								));

							rref = _mm_loadu_ps((sptrbj+*ofs));
							gref = _mm_loadu_ps((sptrgj+*ofs));
							bref = _mm_loadu_ps((sptrrj+*ofs));

							const __m128 _sw = _mm_set1_ps(*spw);//space weight
							__m128 _w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[3]],color_weight[buf[2]],color_weight[buf[1]],color_weight[buf[0]]));//��������̐�Βl����exp��\��LUT�ɓ��Ă�������W�X�^�ɃX�g�A�i�F�d�݁j
							_w = _mm_mul_ps(_w,_mm_loadu_ps(wptrj+wofs[0]));//wmap!

							rref = _mm_mul_ps(_w, rref);
							gref = _mm_mul_ps(_w, gref);
							bref = _mm_mul_ps(_w, bref);

							rval1 = _mm_add_ps(rval1,rref);
							gval1 = _mm_add_ps(gval1,gref);
							bval1 = _mm_add_ps(bval1,bref);
							wval1 = _mm_add_ps(wval1,_w);
						}
						rval1 = _mm_div_ps(rval1,wval1);
						gval1 = _mm_div_ps(gval1,wval1);
						bval1 = _mm_div_ps(bval1,wval1);

						__m128 a = _mm_shuffle_ps(rval1,rval1,_MM_SHUFFLE(3,0,1,2));
						__m128 b = _mm_shuffle_ps(bval1,bval1,_MM_SHUFFLE(1,2,3,0));
						__m128 c = _mm_shuffle_ps(gval1,gval1,_MM_SHUFFLE(2,3,0,1));
						float* dptrc = dptr+3*j;
						_mm_stream_ps((dptrc),_mm_blend_ps(_mm_blend_ps(b,a,4),c,2));
						_mm_stream_ps((dptrc+4),_mm_blend_ps(_mm_blend_ps(c,b,4),a,2));
						_mm_stream_ps((dptrc+8),_mm_blend_ps(_mm_blend_ps(a,c,4),b,2));
					}
				}
#endif
				for(; j < size.width; j++)
				{
					const float* sptrrj = sptrr+j;
					const float* sptrgj = sptrg+j;
					const float* sptrbj = sptrb+j;
					const float* gptrrj = gptrr+j;
					const float* gptrgj = gptrg+j;
					const float* gptrbj = gptrb+j;

					const float r0 = gptrrj[0];
					const float g0 = gptrgj[0];
					const float b0 = gptrbj[0];

					float sum_r=0.0f,sum_b=0.0f,sum_g=0.0f;
					float wsum=0.0f;
					for(k=0 ; k < maxk; k++ )
					{
						const float r = gptrrj[space_guide_ofs[k]], g = gptrgj[space_guide_ofs[k]], b = gptrbj[space_guide_ofs[k]];
						float w = space_weight[k]*color_weight[cvRound(std::abs(b - b0) +std::abs(g - g0) + std::abs(r - r0))];
						sum_b += sptrrj[space_ofs[k]]*w;
						sum_g += sptrgj[space_ofs[k]]*w;
						sum_r += sptrbj[space_ofs[k]]*w;
						wsum += w;
					}
					wsum = 1.f/wsum;
					dptr[3*j  ] = sum_b*wsum;
					dptr[3*j+1] = sum_g*wsum;
					dptr[3*j+2] = sum_r*wsum;
				}
			}

		}
		else if(cn == 3 && cng == 1)
		{
			int CV_DECL_ALIGNED(16) buf[4];

			float* sptrr = (float*)temp->ptr<float>(3*radiusV+3*range.start  ) + 4 * (radiusH/4 + 1);
			float* sptrg = (float*)temp->ptr<float>(3*radiusV+3*range.start+1) + 4 * (radiusH/4 + 1);
			float* sptrb = (float*)temp->ptr<float>(3*radiusV+3*range.start+2) + 4 * (radiusH/4 + 1);
			float* gptr = (float*)guide->ptr<float>(  radiusV+  range.start) + 4 * (radiusH/4 + 1);
			float* wptr = (float*)weightMap->ptr<float>(range.start+radiusV) + 4 * (radiusH/4 + 1);//wmap!

			float* dptr = dest->ptr<float>(range.start);

			const int sstep = 3*temp->cols;
			const int gstep =   guide->cols;
			const int wstep = weightMap->cols;//wmap!

			const int dstep = 3*dest->cols;

			for(i = range.start; i != range.end; i++,gptr+=gstep, sptrr+=sstep,sptrg+=sstep,sptrb+=sstep, dptr+=dstep ,wptr+=wstep)//wmap!
			{	
				j=0;
#if CV_SSE4_1
				if( haveSSE4 )
				{
					for(; j < size.width; j+=4)//4 pixel unit
					{
						int* ofs = &space_ofs[0];
						int* gofs = &space_guide_ofs[0];
						int* wofs = &space_w_ofs[0];//wmap!

						float* spw = space_weight;

						const float* sptrrj = sptrr+j;
						const float* sptrgj = sptrg+j;
						const float* sptrbj = sptrb+j;
						const float* gptrj = gptr+j;
						const float* wptrj = wptr+j;//wmap!

						const __m128 sval = _mm_load_ps((gptrj));

						__m128 wval1 = _mm_set1_ps(0.0f);
						__m128 rval1 = _mm_set1_ps(0.0f);
						__m128 gval1 = _mm_set1_ps(0.0f);
						__m128 bval1 = _mm_set1_ps(0.0f);

						for(k = 0;  k <= maxk; k ++, ofs++,gofs++,spw++,wofs++)//wmap!
						{
							__m128 sref = _mm_loadu_ps((gptrj+*gofs));
							_mm_store_si128((__m128i*)buf,_mm_cvtps_epi32(_mm_and_ps(_mm_sub_ps(sval,sref), *(const __m128*)v32f_absmask)));

							__m128 rref = _mm_loadu_ps((sptrbj+*ofs));
							__m128 gref = _mm_loadu_ps((sptrgj+*ofs));
							__m128 bref = _mm_loadu_ps((sptrrj+*ofs));

							const __m128 _sw = _mm_set1_ps(*spw);//space weight
							__m128 _w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[3]],color_weight[buf[2]],color_weight[buf[1]],color_weight[buf[0]]));//��������̐�Βl����exp��\��LUT�ɓ��Ă�������W�X�^�ɃX�g�A�i�F�d�݁j
							_w = _mm_mul_ps(_w,_mm_loadu_ps(wptrj+wofs[0]));//wmap!
							bref = _mm_mul_ps(_w, bref);
							gref = _mm_mul_ps(_w, gref);
							rref = _mm_mul_ps(_w, rref);

							bval1 = _mm_add_ps(bval1,bref);
							gval1 = _mm_add_ps(gval1,gref);
							rval1 = _mm_add_ps(rval1,rref);
							wval1 = _mm_add_ps(wval1,_w);
						}
						rval1 = _mm_div_ps(rval1,wval1);
						gval1 = _mm_div_ps(gval1,wval1);
						bval1 = _mm_div_ps(bval1,wval1);

						__m128 a = _mm_shuffle_ps(rval1,rval1,_MM_SHUFFLE(3,0,1,2));
						__m128 b = _mm_shuffle_ps(bval1,bval1,_MM_SHUFFLE(1,2,3,0));
						__m128 c = _mm_shuffle_ps(gval1,gval1,_MM_SHUFFLE(2,3,0,1));
						float* dptrc = dptr+3*j;
						_mm_stream_ps((dptrc),_mm_blend_ps(_mm_blend_ps(b,a,4),c,2));
						_mm_stream_ps((dptrc+4),_mm_blend_ps(_mm_blend_ps(c,b,4),a,2));
						_mm_stream_ps((dptrc+8),_mm_blend_ps(_mm_blend_ps(a,c,4),b,2));
					}
				}
#endif
				for(; j < size.width; j++)
				{
					const float* sptrrj = sptrr+j;
					const float* sptrgj = sptrg+j;
					const float* sptrbj = sptrb+j;
					const float* gptrj = gptr+j;

					const float r0 = gptrj[0];
					float sum_r=0.0f,sum_b=0.0f,sum_g=0.0f;
					float wsum=0.0f;
					for(k=0 ; k < maxk; k++ )
					{
						const float r = gptrj[space_guide_ofs[k]];
						float w = space_weight[k]*color_weight[cvRound(std::abs(r - r0))];
						sum_b += sptrrj[space_ofs[k]]*w;
						sum_g += sptrgj[space_ofs[k]]*w;
						sum_r += sptrbj[space_ofs[k]]*w;
						wsum += w;
					}
					wsum = 1.f/wsum;
					dptr[3*j  ] = sum_b*wsum;
					dptr[3*j+1] = sum_g*wsum;
					dptr[3*j+2] = sum_r*wsum;
				}
			}
		}
	}
private:
	const Mat *temp;
	const Mat *weightMap;
	Mat *dest;
	const Mat* guide;
	int radiusH, radiusV, maxk, *space_ofs, *space_guide_ofs,*space_w_ofs;
	float *space_weight, *color_weight;
};

class WeightedJointBilateralFilter_8u_InvokerSSE4 : public cv::ParallelLoopBody
{
public:
	WeightedJointBilateralFilter_8u_InvokerSSE4(Mat& _dest, const Mat& _temp, const Mat& _weightMap,const Mat& _guide, int _radiusH, int _radiusV, int _maxk,
		int* _space_ofs, int* _space_w_ofs,int* _space_guide_ofs, float *_space_weight, float *_color_weight) :
	temp(&_temp), weightMap(&_weightMap),dest(&_dest), guide(&_guide), radiusH(_radiusH), radiusV(_radiusV),
		maxk(_maxk), space_ofs(_space_ofs), space_w_ofs(_space_w_ofs),space_guide_ofs(_space_guide_ofs), space_weight(_space_weight), color_weight(_color_weight)
	{
	}

	virtual void operator() (const Range& range) const
	{
		int i, j, cn = dest->channels(), k;
		int cng = (guide->rows-2*radiusV) / dest->rows;
		Size size = dest->size();

		//imshow("wwww",weightMap);waitKey();
#if CV_SSE4_1
		bool haveSSE4 = checkHardwareSupport(CV_CPU_SSE4_1);
#endif
		if( cn == 1 && cng ==1)
		{
			uchar CV_DECL_ALIGNED(16) buf[16];

			uchar* sptr = (uchar*)temp->ptr(range.start+radiusV) + 16 * (radiusH/16 + 1);
			uchar* gptr = (uchar*)guide->ptr(range.start+radiusV) + 16 * (radiusH/16 + 1);
			float* wptr = (float*)weightMap->ptr<float>(range.start+radiusV)+ 16 * (radiusH/16 + 1);
			uchar* dptr = dest->ptr(range.start);
			const int sstep = temp->cols;
			const int gstep = guide->cols;
			const int dstep = dest->cols;
			const int wstep = weightMap->cols;
			for(i = range.start; i != range.end; i++,dptr+=dstep,sptr+=sstep,gptr+=gstep,wptr+=wstep )
			{
				j=0;
#if CV_SSE4_1
				if( haveSSE4 )
				{
					for(; j < size.width; j+=16)//16 pixel unit
					{
						int* ofs = &space_ofs[0];
						int* gofs = &space_guide_ofs[0];
						int* wofs = &space_w_ofs[0];
						float* spw = space_weight;
						const uchar* sptrj = sptr+j;
						const uchar* gptrj = gptr+j;
						const float* wptrj = wptr+j;

						const __m128i sval = _mm_load_si128((__m128i*)(gptrj));

						__m128 wval1 = _mm_setzero_ps();
						__m128 tval1 = _mm_setzero_ps();
						__m128 wval2 = _mm_setzero_ps();
						__m128 tval2 = _mm_setzero_ps();
						__m128 wval3 = _mm_setzero_ps();
						__m128 tval3 = _mm_setzero_ps();
						__m128 wval4 = _mm_setzero_ps();
						__m128 tval4 = _mm_setzero_ps();
						const __m128i zero = _mm_setzero_si128();
						__m128i m1,m2;
						__m128 _valF,_w;
						for(k = 0;  k <= maxk; k ++, ofs++,wofs++,gofs++,spw++)
						{
							const __m128i sref = _mm_loadu_si128((__m128i*)(gptrj+*gofs));
							_mm_store_si128((__m128i*)buf,_mm_add_epi8(_mm_subs_epu8(sval,sref),_mm_subs_epu8(sref,sval)));

							const __m128i vref = _mm_loadu_si128((__m128i*)(sptrj+*ofs));

							m1 = _mm_unpacklo_epi8(vref,zero);
							m2 = _mm_unpackhi_epi16(m1,zero);
							m1 = _mm_unpacklo_epi16(m1,zero);

							const __m128 _sw = _mm_set1_ps(*spw);
							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[3]],color_weight[buf[2]],color_weight[buf[1]],color_weight[buf[0]]));
							_w = _mm_mul_ps(_w,_mm_loadu_ps(wptrj+wofs[0]));

							_valF = _mm_cvtepi32_ps(m1);
							_valF = _mm_mul_ps(_w, _valF);
							tval1 = _mm_add_ps(tval1,_valF);
							wval1 = _mm_add_ps(wval1,_w);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[7]],color_weight[buf[6]],color_weight[buf[5]],color_weight[buf[4]]));
							_w = _mm_mul_ps(_w,_mm_loadu_ps(wptrj+wofs[0]+4));
							_valF =_mm_cvtepi32_ps(m2);
							_valF = _mm_mul_ps(_w, _valF);
							tval2 = _mm_add_ps(tval2,_valF);
							wval2 = _mm_add_ps(wval2,_w);

							m1 = _mm_unpackhi_epi8(vref,zero);
							m2 = _mm_unpackhi_epi16(m1,zero);
							m1 = _mm_unpacklo_epi16(m1,zero);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[11]],color_weight[buf[10]],color_weight[buf[9]],color_weight[buf[8]]));
							_w = _mm_mul_ps(_w,_mm_loadu_ps(wptrj+wofs[0]+8));
							_valF =_mm_cvtepi32_ps(m1);
							_valF = _mm_mul_ps(_w, _valF);
							wval3 = _mm_add_ps(wval3,_w);
							tval3 = _mm_add_ps(tval3,_valF);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[15]],color_weight[buf[14]],color_weight[buf[13]],color_weight[buf[12]]));
							_w = _mm_mul_ps(_w,_mm_loadu_ps(wptr+j+wofs[0]+12));
							_valF =_mm_cvtepi32_ps(m2);
							_valF = _mm_mul_ps(_w, _valF);
							wval4 = _mm_add_ps(wval4,_w);
							tval4 = _mm_add_ps(tval4,_valF);
						}
						tval1 = _mm_div_ps(tval1,wval1);
						tval2 = _mm_div_ps(tval2,wval2);
						tval3 = _mm_div_ps(tval3,wval3);
						tval4 = _mm_div_ps(tval4,wval4);
						_mm_stream_si128((__m128i*)(dptr+j), _mm_packus_epi16(_mm_packs_epi32( _mm_cvtps_epi32(tval1), _mm_cvtps_epi32(tval2)) , _mm_packs_epi32( _mm_cvtps_epi32(tval3), _mm_cvtps_epi32(tval4))));
					}
				}
#endif
				for(; j < size.width; j++)
				{
					const uchar val0 = gptr[j];
					float sum=0.0f;
					float wsum=0.0f;
					for(k=0 ; k < maxk; k++ )
					{
						int gval = gptr[j + space_guide_ofs[k]];
						int val = sptr[j + space_ofs[k]];
						float w = wptr[j + space_w_ofs[k]]*space_weight[k]*color_weight[std::abs(gval - val0)];
						sum += val*w;
						wsum += w;
					}
					//overflow is not possible here => there is no need to use CV_CAST_8U
					dptr[j] = (uchar)cvRound(sum/wsum);
				}
			}
		}
		else if(cn == 1 && cng == 3)
		{
			assert( cng == 3 );//color
			short CV_DECL_ALIGNED(16) buf[16];

			const int sstep = temp->cols;
			const int gstep = 3*guide->cols;
			const int dstep = dest->cols;
			const int wstep = weightMap->cols;
			uchar* sptr = (uchar*)temp->ptr(range.start+radiusV) + 16 * (radiusH/16 + 1);
			uchar* gptrr = (uchar*)guide->ptr(3*radiusV+3*range.start  ) + 16 * (radiusH/16 + 1);
			uchar* gptrg = (uchar*)guide->ptr(3*radiusV+3*range.start+1) + 16 * (radiusH/16 + 1);
			uchar* gptrb = (uchar*)guide->ptr(3*radiusV+3*range.start+2) + 16 * (radiusH/16 + 1);
			float* wptr = (float*)weightMap->ptr<float>(range.start+radiusV)+ 16 * (radiusH/16 + 1);

			uchar* dptr = dest->ptr(range.start);
			for(i = range.start; i != range.end; i++,gptrr+=gstep,gptrg+=gstep,gptrb+=gstep, sptr+=sstep, dptr+=dstep,wptr+=wstep )
			{	
				j=0;
#if CV_SSE4_1
				if( haveSSE4 )
				{
					for(; j < size.width; j+=16)//16 pixel unit
					{
						__m128i m1,m2,n1,n2;
						__m128 _w, _valF;

						int* ofs = &space_ofs[0];
						int* gofs = &space_guide_ofs[0];
						float* spw = space_weight;
						const float* wptrj = wptr+j;
						int* wofs = &space_w_ofs[0];
						const uchar* sptrj = sptr+j;
						const uchar* gptrrj = gptrr+j;
						const uchar* gptrgj = gptrg+j;
						const uchar* gptrbj = gptrb+j;
						const __m128i bval = _mm_load_si128((__m128i*)(gptrbj));
						const __m128i gval = _mm_load_si128((__m128i*)(gptrgj));
						const __m128i rval = _mm_load_si128((__m128i*)(gptrrj));

						__m128 wval1 = _mm_setzero_ps();
						__m128 tval1 = _mm_setzero_ps();
						__m128 wval2 = _mm_setzero_ps();
						__m128 tval2 = _mm_setzero_ps();
						__m128 wval3 = _mm_setzero_ps();
						__m128 tval3 = _mm_setzero_ps();
						__m128 wval4 = _mm_setzero_ps();
						__m128 tval4 = _mm_setzero_ps();

						const __m128i zero = _mm_setzero_si128();
						for(k = 0;  k <= maxk; k ++, ofs++,gofs++,wofs++,spw++)
						{
							const __m128i bref = _mm_loadu_si128((__m128i*)(gptrbj+*gofs));
							const __m128i gref = _mm_loadu_si128((__m128i*)(gptrgj+*gofs));
							const __m128i rref = _mm_loadu_si128((__m128i*)(gptrrj+*gofs));

							m1 = _mm_add_epi8(_mm_subs_epu8(rval,rref),_mm_subs_epu8(rref,rval));
							m2 = _mm_unpackhi_epi8(m1,zero);
							m1 = _mm_unpacklo_epi8(m1,zero);

							n1 = _mm_add_epi8(_mm_subs_epu8(gval,gref),_mm_subs_epu8(gref,gval));
							n2 = _mm_unpackhi_epi8(n1,zero);
							n1 = _mm_unpacklo_epi8(n1,zero);

							m1 = _mm_add_epi16(m1,n1);
							m2 = _mm_add_epi16(m2,n2);

							n1 = _mm_add_epi8(_mm_subs_epu8(bval,bref),_mm_subs_epu8(bref,bval));
							n2 = _mm_unpackhi_epi8(n1,zero);
							n1 = _mm_unpacklo_epi8(n1,zero);

							m1 = _mm_add_epi16(m1,n1);
							m2 = _mm_add_epi16(m2,n2);

							_mm_store_si128((__m128i*)(buf+8),m2);
							_mm_store_si128((__m128i*)buf,m1);

							const __m128i vref = _mm_loadu_si128((__m128i*)(sptrj+*ofs));

							m1 = _mm_unpacklo_epi8(vref,zero);
							m2 = _mm_unpackhi_epi16(m1,zero);
							m1 = _mm_unpacklo_epi16(m1,zero);

							const __m128 _sw = _mm_set1_ps(*spw);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[3]],color_weight[buf[2]],color_weight[buf[1]],color_weight[buf[0]]));
							_w = _mm_mul_ps(_w,_mm_loadu_ps(wptrj+wofs[0]));
							_valF = _mm_cvtepi32_ps(m1);
							_valF = _mm_mul_ps(_w, _valF);
							tval1 = _mm_add_ps(tval1,_valF);
							wval1 = _mm_add_ps(wval1,_w);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[7]],color_weight[buf[6]],color_weight[buf[5]],color_weight[buf[4]]));
							_w = _mm_mul_ps(_w, _mm_loadu_ps(wptr+j+wofs[0]+4));
							_valF =_mm_cvtepi32_ps(m2);
							_valF = _mm_mul_ps(_w, _valF);
							tval2 = _mm_add_ps(tval2,_valF);
							wval2 = _mm_add_ps(wval2,_w);


							m1 = _mm_unpackhi_epi8(vref,zero);
							m2 = _mm_unpackhi_epi16(m1,zero);
							m1 = _mm_unpacklo_epi16(m1,zero);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[11]],color_weight[buf[10]],color_weight[buf[9]],color_weight[buf[8]]));
							_w = _mm_mul_ps(_w, _mm_loadu_ps(wptr+j+wofs[0]+8));
							_valF =_mm_cvtepi32_ps(m1);
							_valF = _mm_mul_ps(_w, _valF);
							wval3 = _mm_add_ps(wval3,_w);
							tval3 = _mm_add_ps(tval3,_valF);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[15]],color_weight[buf[14]],color_weight[buf[13]],color_weight[buf[12]]));
							_w = _mm_mul_ps(_w, _mm_loadu_ps(wptr+j+wofs[0]+12));
							_valF =_mm_cvtepi32_ps(m2);
							_valF = _mm_mul_ps(_w, _valF);
							wval4 = _mm_add_ps(wval4,_w);
							tval4 = _mm_add_ps(tval4,_valF);
						}
						tval1 = _mm_div_ps(tval1,wval1);
						tval2 = _mm_div_ps(tval2,wval2);
						tval3 = _mm_div_ps(tval3,wval3);
						tval4 = _mm_div_ps(tval4,wval4);
						_mm_store_si128((__m128i*)(dptr+j), _mm_packus_epi16(_mm_packs_epi32( _mm_cvtps_epi32(tval1), _mm_cvtps_epi32(tval2)) , _mm_packs_epi32( _mm_cvtps_epi32(tval3), _mm_cvtps_epi32(tval4))));
					}
				}
#endif
				for(; j < size.width; j++)
				{
					const uchar* sptrj = sptr+j;
					const uchar* gptrrj = gptrr+j;
					const uchar* gptrgj = gptrg+j;
					const uchar* gptrbj = gptrb+j;

					int r0 = gptrrj[0];
					int g0 = gptrgj[0];
					int b0 = gptrbj[0];

					float sum=0.0f;
					float wsum=0.0f;
					for(k=0 ; k < maxk; k++ )
					{
						int r = gptrrj[space_guide_ofs[k]], g = gptrgj[space_guide_ofs[k]], b = gptrbj[space_guide_ofs[k]];
						float w = wptr[j+space_w_ofs[k]]*space_weight[k]*color_weight[std::abs(b - b0) +std::abs(g - g0) + std::abs(r - r0)];
						sum += sptrj[space_ofs[k]]*w;
						wsum += w;
					}
					//overflow is not possible here => there is no need to use CV_CAST_8U
					dptr[j] = (uchar)cvRound(sum/wsum);
				}
			}
		}
		else if(cn == 3 && cng == 3)
		{
			assert( cng == 3 );//color
			short CV_DECL_ALIGNED(16) buf[16];

			const int sstep = 3*temp->cols;
			const int gstep = 3*guide->cols;
			const int dstep = 3*dest->cols;

			const int wstep = weightMap->cols;
			uchar* sptrr =  (uchar*)temp->ptr(3*radiusV+3*range.start  ) + 16 * (radiusH/16 + 1);
			uchar* sptrg =  (uchar*)temp->ptr(3*radiusV+3*range.start+1) + 16 * (radiusH/16 + 1);
			uchar* sptrb =  (uchar*)temp->ptr(3*radiusV+3*range.start+2) + 16 * (radiusH/16 + 1);
			uchar* gptrr = (uchar*)guide->ptr(3*radiusV+3*range.start  ) + 16 * (radiusH/16 + 1);
			uchar* gptrg = (uchar*)guide->ptr(3*radiusV+3*range.start+1) + 16 * (radiusH/16 + 1);
			uchar* gptrb = (uchar*)guide->ptr(3*radiusV+3*range.start+2) + 16 * (radiusH/16 + 1);
			float* wptr = (float*)weightMap->ptr<float>(range.start+radiusV)+ 16 * (radiusH/16 + 1);
			uchar* dptr = dest->ptr(range.start);
			for(i = range.start; i != range.end; i++,gptrr+=gstep,gptrg+=gstep,gptrb+=gstep, sptrr+=sstep,sptrg+=sstep,sptrb+=sstep, dptr+=dstep, wptr+=wstep )
			{	
				j=0;
#if CV_SSE4_1
				if( haveSSE4 )
				{
					for(; j < size.width; j+=16)//16 pixel unit
					{
						__m128 _w, _valr,_valg,_valb;

						int* ofs = &space_ofs[0];
						int* gofs = &space_guide_ofs[0];
						int* wofs = &space_w_ofs[0];
						float* spw = space_weight;
						const float* wptrj = wptr+j;
						const uchar* sptrrj = sptrr+j;
						const uchar* sptrgj = sptrg+j;
						const uchar* sptrbj = sptrb+j;
						const uchar* gptrrj = gptrr+j;
						const uchar* gptrgj = gptrg+j;
						const uchar* gptrbj = gptrb+j;
						const __m128i bval = _mm_load_si128((__m128i*)(gptrbj));
						const __m128i gval = _mm_load_si128((__m128i*)(gptrgj));
						const __m128i rval = _mm_load_si128((__m128i*)(gptrrj));

						__m128 wval1 = _mm_set1_ps(0.0f);
						__m128 rval1 = _mm_set1_ps(0.0f);
						__m128 gval1 = _mm_set1_ps(0.0f);
						__m128 bval1 = _mm_set1_ps(0.0f);

						__m128 wval2 = _mm_set1_ps(0.0f);
						__m128 rval2 = _mm_set1_ps(0.0f);
						__m128 gval2 = _mm_set1_ps(0.0f);
						__m128 bval2 = _mm_set1_ps(0.0f);

						__m128 wval3 = _mm_set1_ps(0.0f);
						__m128 rval3 = _mm_set1_ps(0.0f);
						__m128 gval3 = _mm_set1_ps(0.0f);
						__m128 bval3 = _mm_set1_ps(0.0f);

						__m128 wval4 = _mm_set1_ps(0.0f);
						__m128 rval4 = _mm_set1_ps(0.0f);
						__m128 gval4 = _mm_set1_ps(0.0f);
						__m128 bval4 = _mm_set1_ps(0.0f);

						const __m128i zero = _mm_setzero_si128();

						for(k = 0;  k <= maxk; k ++, ofs++,gofs++,wofs++,spw++)
						{
							__m128i bref = _mm_loadu_si128((__m128i*)(gptrbj+*gofs));
							__m128i gref = _mm_loadu_si128((__m128i*)(gptrgj+*gofs));
							__m128i rref = _mm_loadu_si128((__m128i*)(gptrrj+*gofs));

							__m128i r1 = _mm_add_epi8(_mm_subs_epu8(rval,rref),_mm_subs_epu8(rref,rval));
							__m128i r2 = _mm_unpackhi_epi8(r1,zero);
							r1 = _mm_unpacklo_epi8(r1,zero);

							__m128i g1 = _mm_add_epi8(_mm_subs_epu8(gval,gref),_mm_subs_epu8(gref,gval));
							__m128i g2 = _mm_unpackhi_epi8(g1,zero);
							g1 = _mm_unpacklo_epi8(g1,zero);

							r1 = _mm_add_epi16(r1,g1);
							r2 = _mm_add_epi16(r2,g2);

							__m128i b1 = _mm_add_epi8(_mm_subs_epu8(bval,bref),_mm_subs_epu8(bref,bval));
							__m128i b2 = _mm_unpackhi_epi8(b1,zero);
							b1 = _mm_unpacklo_epi8(b1,zero);

							r1 = _mm_add_epi16(r1,b1);
							r2 = _mm_add_epi16(r2,b2);

							_mm_store_si128((__m128i*)(buf+8),r2);
							_mm_store_si128((__m128i*)buf,r1);

							bref = _mm_loadu_si128((__m128i*)(sptrbj+*ofs));
							gref = _mm_loadu_si128((__m128i*)(sptrgj+*ofs));
							rref = _mm_loadu_si128((__m128i*)(sptrrj+*ofs));

							r1 = _mm_unpacklo_epi8(rref,zero);
							r2 = _mm_unpackhi_epi16(r1,zero);
							r1 = _mm_unpacklo_epi16(r1,zero);
							g1 = _mm_unpacklo_epi8(gref,zero);
							g2 = _mm_unpackhi_epi16(g1,zero);
							g1 = _mm_unpacklo_epi16(g1,zero);
							b1 = _mm_unpacklo_epi8(bref,zero);
							b2 = _mm_unpackhi_epi16(b1,zero);
							b1 = _mm_unpacklo_epi16(b1,zero);

							const __m128 _sw = _mm_set1_ps(*spw);
							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[3]],color_weight[buf[2]],color_weight[buf[1]],color_weight[buf[0]]));
							_w = _mm_mul_ps(_w, _mm_loadu_ps(wptr+j+wofs[0]));

							_valr = _mm_cvtepi32_ps(r1);
							_valg = _mm_cvtepi32_ps(g1);
							_valb = _mm_cvtepi32_ps(b1);

							_valr = _mm_mul_ps(_w, _valr);
							_valg = _mm_mul_ps(_w, _valg);
							_valb = _mm_mul_ps(_w, _valb);

							rval1 = _mm_add_ps(rval1,_valr);
							gval1 = _mm_add_ps(gval1,_valg);
							bval1 = _mm_add_ps(bval1,_valb);
							wval1 = _mm_add_ps(wval1,_w);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[7]],color_weight[buf[6]],color_weight[buf[5]],color_weight[buf[4]]));
							_w = _mm_mul_ps(_w, _mm_loadu_ps(wptr+j+wofs[0]+4));
							_valr =_mm_cvtepi32_ps(r2);
							_valg =_mm_cvtepi32_ps(g2);
							_valb =_mm_cvtepi32_ps(b2);

							_valr = _mm_mul_ps(_w, _valr);
							_valg = _mm_mul_ps(_w, _valg);
							_valb = _mm_mul_ps(_w, _valb);

							rval2 = _mm_add_ps(rval2,_valr);
							gval2 = _mm_add_ps(gval2,_valg);
							bval2 = _mm_add_ps(bval2,_valb);
							wval2 = _mm_add_ps(wval2,_w);

							r1 = _mm_unpackhi_epi8(rref,zero);
							r2 = _mm_unpackhi_epi16(r1,zero);
							r1 = _mm_unpacklo_epi16(r1,zero);

							g1 = _mm_unpackhi_epi8(gref,zero);
							g2 = _mm_unpackhi_epi16(g1,zero);
							g1 = _mm_unpacklo_epi16(g1,zero);

							b1 = _mm_unpackhi_epi8(bref,zero);
							b2 = _mm_unpackhi_epi16(b1,zero);
							b1 = _mm_unpacklo_epi16(b1,zero);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[11]],color_weight[buf[10]],color_weight[buf[9]],color_weight[buf[8]]));
							_w = _mm_mul_ps(_w, _mm_loadu_ps(wptr+j+wofs[0]+8));
							_valr =_mm_cvtepi32_ps(r1);
							_valg =_mm_cvtepi32_ps(g1);
							_valb =_mm_cvtepi32_ps(b1);

							_valr = _mm_mul_ps(_w, _valr);
							_valg = _mm_mul_ps(_w, _valg);
							_valb = _mm_mul_ps(_w, _valb);

							wval3 = _mm_add_ps(wval3,_w);
							rval3 = _mm_add_ps(rval3,_valr);
							gval3 = _mm_add_ps(gval3,_valg);
							bval3 = _mm_add_ps(bval3,_valb);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[15]],color_weight[buf[14]],color_weight[buf[13]],color_weight[buf[12]]));
							_w = _mm_mul_ps(_w, _mm_loadu_ps(wptr+j+wofs[0]+12));
							_valr =_mm_cvtepi32_ps(r2);
							_valg =_mm_cvtepi32_ps(g2);
							_valb =_mm_cvtepi32_ps(b2);

							_valr = _mm_mul_ps(_w, _valr);
							_valg = _mm_mul_ps(_w, _valg);
							_valb = _mm_mul_ps(_w, _valb);

							wval4 = _mm_add_ps(wval4,_w);
							rval4 = _mm_add_ps(rval4,_valr);
							gval4 = _mm_add_ps(gval4,_valg);
							bval4 = _mm_add_ps(bval4,_valb);	
						}
						rval1 = _mm_div_ps(rval1,wval1);
						rval2 = _mm_div_ps(rval2,wval2);
						rval3 = _mm_div_ps(rval3,wval3);
						rval4 = _mm_div_ps(rval4,wval4);
						__m128i a = _mm_packus_epi16(_mm_packs_epi32( _mm_cvtps_epi32(rval1), _mm_cvtps_epi32(rval2)) , _mm_packs_epi32( _mm_cvtps_epi32(rval3), _mm_cvtps_epi32(rval4)));
						gval1 = _mm_div_ps(gval1,wval1);
						gval2 = _mm_div_ps(gval2,wval2);
						gval3 = _mm_div_ps(gval3,wval3);
						gval4 = _mm_div_ps(gval4,wval4);
						__m128i b = _mm_packus_epi16(_mm_packs_epi32( _mm_cvtps_epi32(gval1), _mm_cvtps_epi32(gval2)) , _mm_packs_epi32( _mm_cvtps_epi32(gval3), _mm_cvtps_epi32(gval4)));
						bval1 = _mm_div_ps(bval1,wval1);
						bval2 = _mm_div_ps(bval2,wval2);
						bval3 = _mm_div_ps(bval3,wval3);
						bval4 = _mm_div_ps(bval4,wval4);
						__m128i c = _mm_packus_epi16(_mm_packs_epi32( _mm_cvtps_epi32(bval1), _mm_cvtps_epi32(bval2)) , _mm_packs_epi32( _mm_cvtps_epi32(bval3), _mm_cvtps_epi32(bval4)));

						//sse4///
						uchar* dptrc = dptr+3*j;
						const __m128i mask1 = _mm_setr_epi8(0, 11, 6, 1, 12, 7, 2, 13, 8, 3, 14, 9, 4, 15, 10, 5);
						const __m128i mask2 = _mm_setr_epi8(5, 0, 11, 6, 1, 12, 7, 2, 13, 8, 3, 14, 9, 4, 15, 10);
						const __m128i mask3 = _mm_setr_epi8(10, 5, 0, 11, 6, 1, 12, 7, 2, 13, 8, 3, 14, 9, 4, 15);
						const __m128i bmask1 = _mm_setr_epi8(0,255,255,0,255,255,0,255,255,0,255,255,0,255,255,0);
						const __m128i bmask2 = _mm_setr_epi8(255,255,0,255,255,0,255,255,0,255,255,0,255,255,0,255);

						a = _mm_shuffle_epi8(a,mask1);
						b = _mm_shuffle_epi8(b,mask2);
						c = _mm_shuffle_epi8(c,mask3);
						_mm_stream_si128((__m128i*)(dptrc),_mm_blendv_epi8(c,_mm_blendv_epi8(a,b,bmask1),bmask2));
						_mm_stream_si128((__m128i*)(dptrc+16),_mm_blendv_epi8(b,_mm_blendv_epi8(a,c,bmask2),bmask1));		
						_mm_stream_si128((__m128i*)(dptrc+32),_mm_blendv_epi8(c,_mm_blendv_epi8(b,a,bmask2),bmask1));
					}
				}
#endif
				for(; j < size.width; j++)
				{
					const uchar* sptrrj = sptrr+j;
					const uchar* sptrgj = sptrg+j;
					const uchar* sptrbj = sptrb+j;
					const uchar* gptrrj = gptrr+j;
					const uchar* gptrgj = gptrg+j;
					const uchar* gptrbj = gptrb+j;

					int r0 = gptrrj[0];
					int g0 = gptrgj[0];
					int b0 = gptrbj[0];

					float sum_r=0.0f,sum_b=0.0f,sum_g=0.0f;
					float wsum=0.0f;
					for(k=0 ; k < maxk; k++ )
					{
						int r = gptrrj[space_guide_ofs[k]], g = gptrgj[space_guide_ofs[k]], b = gptrbj[space_guide_ofs[k]];
						float w = wptr[j+space_w_ofs[k]]*space_weight[k]*color_weight[std::abs(b - b0) +std::abs(g - g0) + std::abs(r - r0)];
						sum_b += sptrrj[space_ofs[k]]*w;
						sum_g += sptrgj[space_ofs[k]]*w;
						sum_r += sptrbj[space_ofs[k]]*w;
						wsum += w;
					}
					//overflow is not possible here => there is no need to use CV_CAST_8U

					wsum = 1.f/wsum;
					b0 = cvRound(sum_b*wsum);
					g0 = cvRound(sum_g*wsum);
					r0 = cvRound(sum_r*wsum);
					dptr[3*j] = (uchar)b0; dptr[3*j+1] = (uchar)g0; dptr[3*j+2] = (uchar)r0;
				}
			}

		}
		else if(cn == 3 && cng == 1)
		{
			uchar CV_DECL_ALIGNED(16) buf[16];

			const int sstep = 3*temp->cols;
			const int gstep =   guide->cols;
			const int dstep = 3*dest->cols;

			const int wstep = weightMap->cols;
			uchar* sptrr = (uchar*)temp->ptr(3*radiusV+3*range.start  ) + 16 * (radiusH/16 + 1);
			uchar* sptrg = (uchar*)temp->ptr(3*radiusV+3*range.start+1) + 16 * (radiusH/16 + 1);
			uchar* sptrb = (uchar*)temp->ptr(3*radiusV+3*range.start+2) + 16 * (radiusH/16 + 1);
			uchar* gptr = (uchar*)guide->ptr(  radiusV+  range.start) + 16 * (radiusH/16 + 1);
			uchar* dptr = dest->ptr(range.start);
			float* wptr = (float*)weightMap->ptr<float>(range.start+radiusV)+ 16 * (radiusH/16 + 1);
			for(i = range.start; i != range.end; i++,gptr+=gstep, sptrr+=sstep,sptrg+=sstep,sptrb+=sstep, dptr+=dstep,wptr+=wstep )
			{	
				j=0;
#if CV_SSE4_1
				if( haveSSE4 )
				{
					for(; j < size.width; j+=16)//16 pixel unit
					{

						int* ofs = &space_ofs[0];
						int* gofs = &space_guide_ofs[0];
						float* spw = space_weight;
						int* wofs = &space_w_ofs[0];
						const float* wptrj = wptr+j;
						const uchar* sptrrj = sptrr+j;
						const uchar* sptrgj = sptrg+j;
						const uchar* sptrbj = sptrb+j;
						const uchar* gptrj = gptr+j;
						const __m128i sval = _mm_load_si128((__m128i*)(gptrj));

						__m128 wval1 = _mm_set1_ps(0.0f);
						__m128 rval1 = _mm_set1_ps(0.0f);
						__m128 gval1 = _mm_set1_ps(0.0f);
						__m128 bval1 = _mm_set1_ps(0.0f);

						__m128 wval2 = _mm_set1_ps(0.0f);
						__m128 rval2 = _mm_set1_ps(0.0f);
						__m128 gval2 = _mm_set1_ps(0.0f);
						__m128 bval2 = _mm_set1_ps(0.0f);

						__m128 wval3 = _mm_set1_ps(0.0f);
						__m128 rval3 = _mm_set1_ps(0.0f);
						__m128 gval3 = _mm_set1_ps(0.0f);
						__m128 bval3 = _mm_set1_ps(0.0f);

						__m128 wval4 = _mm_set1_ps(0.0f);
						__m128 rval4 = _mm_set1_ps(0.0f);
						__m128 gval4 = _mm_set1_ps(0.0f);
						__m128 bval4 = _mm_set1_ps(0.0f);

						const __m128i zero = _mm_setzero_si128();
						for(k = 0;  k <= maxk; k ++, ofs++,gofs++,wofs++,spw++)
						{
							__m128i sref = _mm_loadu_si128((__m128i*)(gptrj+*gofs));
							_mm_store_si128((__m128i*)buf,_mm_add_epi8(_mm_subs_epu8(sval,sref),_mm_subs_epu8(sref,sval)));

							__m128i bref = _mm_loadu_si128((__m128i*)(sptrbj+*ofs));
							__m128i gref = _mm_loadu_si128((__m128i*)(sptrgj+*ofs));
							__m128i rref = _mm_loadu_si128((__m128i*)(sptrrj+*ofs));

							__m128i r1 = _mm_unpacklo_epi8(rref,zero);
							__m128i r2 = _mm_unpackhi_epi16(r1,zero);
							r1 = _mm_unpacklo_epi16(r1,zero);
							__m128i g1 = _mm_unpacklo_epi8(gref,zero);
							__m128i g2 = _mm_unpackhi_epi16(g1,zero);
							g1 = _mm_unpacklo_epi16(g1,zero);
							__m128i b1 = _mm_unpacklo_epi8(bref,zero);
							__m128i b2 = _mm_unpackhi_epi16(b1,zero);
							b1 = _mm_unpacklo_epi16(b1,zero);

							const __m128 _sw = _mm_set1_ps(*spw);
							__m128 _w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[3]],color_weight[buf[2]],color_weight[buf[1]],color_weight[buf[0]]));
							_w = _mm_mul_ps(_w, _mm_loadu_ps(wptr+j+wofs[0]));

							__m128 _valr = _mm_cvtepi32_ps(r1);
							__m128 _valg = _mm_cvtepi32_ps(g1);
							__m128 _valb = _mm_cvtepi32_ps(b1);

							_valr = _mm_mul_ps(_w, _valr);
							_valg = _mm_mul_ps(_w, _valg);
							_valb = _mm_mul_ps(_w, _valb);

							rval1 = _mm_add_ps(rval1,_valr);
							gval1 = _mm_add_ps(gval1,_valg);
							bval1 = _mm_add_ps(bval1,_valb);
							wval1 = _mm_add_ps(wval1,_w);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[7]],color_weight[buf[6]],color_weight[buf[5]],color_weight[buf[4]]));
							_w = _mm_mul_ps(_w, _mm_loadu_ps(wptr+j+wofs[0]+4));
							_valr =_mm_cvtepi32_ps(r2);
							_valg =_mm_cvtepi32_ps(g2);
							_valb =_mm_cvtepi32_ps(b2);

							_valr = _mm_mul_ps(_w, _valr);
							_valg = _mm_mul_ps(_w, _valg);
							_valb = _mm_mul_ps(_w, _valb);

							rval2 = _mm_add_ps(rval2,_valr);
							gval2 = _mm_add_ps(gval2,_valg);
							bval2 = _mm_add_ps(bval2,_valb);
							wval2 = _mm_add_ps(wval2,_w);

							r1 = _mm_unpackhi_epi8(rref,zero);
							r2 = _mm_unpackhi_epi16(r1,zero);
							r1 = _mm_unpacklo_epi16(r1,zero);

							g1 = _mm_unpackhi_epi8(gref,zero);
							g2 = _mm_unpackhi_epi16(g1,zero);
							g1 = _mm_unpacklo_epi16(g1,zero);

							b1 = _mm_unpackhi_epi8(bref,zero);
							b2 = _mm_unpackhi_epi16(b1,zero);
							b1 = _mm_unpacklo_epi16(b1,zero);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[11]],color_weight[buf[10]],color_weight[buf[9]],color_weight[buf[8]]));
							_w = _mm_mul_ps(_w, _mm_loadu_ps(wptr+j+wofs[0]+8));
							_valr =_mm_cvtepi32_ps(r1);
							_valg =_mm_cvtepi32_ps(g1);
							_valb =_mm_cvtepi32_ps(b1);

							_valr = _mm_mul_ps(_w, _valr);
							_valg = _mm_mul_ps(_w, _valg);
							_valb = _mm_mul_ps(_w, _valb);

							wval3 = _mm_add_ps(wval3,_w);
							rval3 = _mm_add_ps(rval3,_valr);
							gval3 = _mm_add_ps(gval3,_valg);
							bval3 = _mm_add_ps(bval3,_valb);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[15]],color_weight[buf[14]],color_weight[buf[13]],color_weight[buf[12]]));
							_w = _mm_mul_ps(_w, _mm_loadu_ps(wptr+j+wofs[0]+12));
							_valr =_mm_cvtepi32_ps(r2);
							_valg =_mm_cvtepi32_ps(g2);
							_valb =_mm_cvtepi32_ps(b2);

							_valr = _mm_mul_ps(_w, _valr);
							_valg = _mm_mul_ps(_w, _valg);
							_valb = _mm_mul_ps(_w, _valb);

							wval4 = _mm_add_ps(wval4,_w);
							rval4 = _mm_add_ps(rval4,_valr);
							gval4 = _mm_add_ps(gval4,_valg);
							bval4 = _mm_add_ps(bval4,_valb);	
						}
						rval1 = _mm_div_ps(rval1,wval1);
						rval2 = _mm_div_ps(rval2,wval2);
						rval3 = _mm_div_ps(rval3,wval3);
						rval4 = _mm_div_ps(rval4,wval4);
						__m128i a = _mm_packus_epi16(_mm_packs_epi32( _mm_cvtps_epi32(rval1), _mm_cvtps_epi32(rval2)) , _mm_packs_epi32( _mm_cvtps_epi32(rval3), _mm_cvtps_epi32(rval4)));
						gval1 = _mm_div_ps(gval1,wval1);
						gval2 = _mm_div_ps(gval2,wval2);
						gval3 = _mm_div_ps(gval3,wval3);
						gval4 = _mm_div_ps(gval4,wval4);
						__m128i b = _mm_packus_epi16(_mm_packs_epi32( _mm_cvtps_epi32(gval1), _mm_cvtps_epi32(gval2)) , _mm_packs_epi32( _mm_cvtps_epi32(gval3), _mm_cvtps_epi32(gval4)));
						bval1 = _mm_div_ps(bval1,wval1);
						bval2 = _mm_div_ps(bval2,wval2);
						bval3 = _mm_div_ps(bval3,wval3);
						bval4 = _mm_div_ps(bval4,wval4);
						__m128i c = _mm_packus_epi16(_mm_packs_epi32( _mm_cvtps_epi32(bval1), _mm_cvtps_epi32(bval2)) , _mm_packs_epi32( _mm_cvtps_epi32(bval3), _mm_cvtps_epi32(bval4)));

						//sse4///
						uchar* dptrc = dptr+3*j;
						const __m128i mask1 = _mm_setr_epi8(0, 11, 6, 1, 12, 7, 2, 13, 8, 3, 14, 9, 4, 15, 10, 5);
						const __m128i mask2 = _mm_setr_epi8(5, 0, 11, 6, 1, 12, 7, 2, 13, 8, 3, 14, 9, 4, 15, 10);
						const __m128i mask3 = _mm_setr_epi8(10, 5, 0, 11, 6, 1, 12, 7, 2, 13, 8, 3, 14, 9, 4, 15);

						const __m128i bmask1 = _mm_setr_epi8
							(0,255,255,0,255,255,0,255,255,0,255,255,0,255,255,0);

						const __m128i bmask2 = _mm_setr_epi8
							(255,255,0,255,255,0,255,255,0,255,255,0,255,255,0,255);

						a = _mm_shuffle_epi8(a,mask1);
						b = _mm_shuffle_epi8(b,mask2);
						c = _mm_shuffle_epi8(c,mask3);

						_mm_stream_si128((__m128i*)(dptrc),_mm_blendv_epi8(c,_mm_blendv_epi8(a,b,bmask1),bmask2));
						_mm_stream_si128((__m128i*)(dptrc+16),_mm_blendv_epi8(b,_mm_blendv_epi8(a,c,bmask2),bmask1));		
						_mm_stream_si128((__m128i*)(dptrc+32),_mm_blendv_epi8(c,_mm_blendv_epi8(b,a,bmask2),bmask1));
					}
				}
#endif
				for(; j < size.width; j++)
				{
					const uchar* sptrrj = sptrr+j;
					const uchar* sptrgj = sptrg+j;
					const uchar* sptrbj = sptrb+j;
					const uchar* gptrj = gptr+j;

					int r0 = gptrj[0];

					float sum_r=0.0f,sum_b=0.0f,sum_g=0.0f;
					float wsum=0.0f;
					for(k=0 ; k < maxk; k++ )
					{
						int r = gptrj[space_guide_ofs[k]];
						float w = wptr[space_w_ofs[k]]*space_weight[k]*color_weight[std::abs(r - r0)];
						sum_b += sptrrj[space_ofs[k]]*w;
						sum_g += sptrgj[space_ofs[k]]*w;
						sum_r += sptrbj[space_ofs[k]]*w;
						wsum += w;
					}
					//overflow is not possible here => there is no need to use CV_CAST_8U

					wsum = 1.f/wsum;
					int b0 = cvRound(sum_b*wsum);
					int g0 = cvRound(sum_g*wsum);
					r0 = cvRound(sum_r*wsum);
					dptr[3*j] = (uchar)b0; dptr[3*j+1] = (uchar)g0; dptr[3*j+2] = (uchar)r0;
				}
			}
		}
	}
private:
	const Mat *temp;
	const Mat *weightMap;
	Mat *dest;
	const Mat* guide;
	int radiusH, radiusV, maxk, *space_ofs, *space_guide_ofs,*space_w_ofs;
	float *space_weight, *color_weight;
};


class JointBilateralFilter_32f_InvokerSSE4 : public cv::ParallelLoopBody
{
public:
	JointBilateralFilter_32f_InvokerSSE4(Mat& _dest, const Mat& _temp, const Mat& _guide, int _radiusH, int _radiusV, int _maxk,
		int* _space_ofs, int* _space_guide_ofs, float *_space_weight, float *_color_weight) :
	temp(&_temp), dest(&_dest), guide(&_guide), radiusH(_radiusH), radiusV(_radiusV),
		maxk(_maxk), space_ofs(_space_ofs), space_guide_ofs(_space_guide_ofs), space_weight(_space_weight), color_weight(_color_weight)
	{
	}

	virtual void operator() (const Range& range) const
	{
		int i, j, cn = dest->channels(), k;
		int cng = (guide->rows-2*radiusV) / dest->rows;
		Size size = dest->size();

		static int CV_DECL_ALIGNED(16) v32f_absmask[] = { 0x7fffffff, 0x7fffffff, 0x7fffffff, 0x7fffffff };

#if CV_SSE4_1
		bool haveSSE4 = checkHardwareSupport(CV_CPU_SSE4_1);
#endif
		if( cn == 1 && cng ==1)
		{
			int CV_DECL_ALIGNED(16) buf[4];

			float* sptr = (float*)temp->ptr<float>(range.start+radiusV) + 4 * (radiusH/4 + 1);
			float* gptr = (float*)guide->ptr<float>(range.start+radiusV) + 4 * (radiusH/4 + 1);
			float* dptr = dest->ptr<float>(range.start);
			const int sstep = temp->cols;
			const int gstep = guide->cols;
			const int dstep = dest->cols;
			for(i = range.start; i != range.end; i++,dptr+=dstep,sptr+=sstep,gptr+=gstep )
			{
				j=0;
#if CV_SSE4_1
				if( haveSSE4 )
				{
					for(; j < size.width; j+=4)//4 pixel unit 
					{
						int* ofs = &space_ofs[0];
						int* gofs = &space_guide_ofs[0];
						float* spw = space_weight;
						const float* sptrj = sptr+j;
						const float* gptrj = gptr+j;
						const __m128 sval = _mm_load_ps((gptrj));
						
						__m128 wval1 = _mm_setzero_ps();
						__m128 tval1 = _mm_setzero_ps();

						for(k = 0;  k <= maxk; k ++, ofs++,gofs++,spw++)
						{
							__m128 sref = _mm_loadu_ps((gptrj+*gofs));
							_mm_store_si128((__m128i*)buf,_mm_cvtps_epi32(_mm_and_ps(_mm_sub_ps(sval,sref), *(const __m128*)v32f_absmask)));

							__m128 vref = _mm_loadu_ps((sptrj+*ofs));
							const __m128 _sw = _mm_set1_ps(*spw);
							__m128 _w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[3]],color_weight[buf[2]],color_weight[buf[1]],color_weight[buf[0]]));
							vref = _mm_mul_ps(_w, vref);
							tval1 = _mm_add_ps(tval1,vref);
							wval1 = _mm_add_ps(wval1,_w);
						}
						tval1 = _mm_div_ps(tval1,wval1);
						_mm_stream_ps((dptr+j), tval1);
					}
				}
#endif
				for(; j < size.width; j++)
				{
					const float val0 = gptr[j];
					float sum=0.0f;
					float wsum=0.0f;
					for(k=0 ; k < maxk; k++ )
					{
						float gval = gptr[j + space_guide_ofs[k]];
						float val = sptr[j + space_ofs[k]];
						float w = space_weight[k]*color_weight[cvRound(std::abs(gval - val0))];
						sum += val*w;
						wsum += w;
					}
					dptr[j] = sum/wsum;
				}
			}
		}
		else if(cn == 1 && cng == 3)
		{
			assert( cng == 3 );//color
			int CV_DECL_ALIGNED(16) buf[4];

			const int sstep = temp->cols;
			const int gstep = 3*guide->cols;
			const int dstep = dest->cols;

			float* sptr = (float*)temp->ptr<float>(range.start+radiusV) + 4 * (radiusH/4 + 1);
			float* gptrr = (float*)guide->ptr<float>(3*radiusV+3*range.start  ) + 4 * (radiusH/4 + 1);
			float* gptrg = (float*)guide->ptr<float>(3*radiusV+3*range.start+1) + 4 * (radiusH/4 + 1);
			float* gptrb = (float*)guide->ptr<float>(3*radiusV+3*range.start+2) + 4 * (radiusH/4 + 1);

			float* dptr = dest->ptr<float>(range.start);
			for(i = range.start; i != range.end; i++,gptrr+=gstep,gptrg+=gstep,gptrb+=gstep, sptr+=sstep, dptr+=dstep )
			{	
				j=0;
#if CV_SSE4_1
				if( haveSSE4 )
				{
					for(; j < size.width; j+=4)//4 pixel unit
					{
						int* ofs = &space_ofs[0];
						int* gofs = &space_guide_ofs[0];
						float* spw = space_weight;
						const float* sptrj = sptr+j;
						const float* gptrrj = gptrr+j;
						const float* gptrgj = gptrg+j;
						const float* gptrbj = gptrb+j;
						const __m128 bval = _mm_load_ps((gptrbj));
						const __m128 gval = _mm_load_ps((gptrgj));
						const __m128 rval = _mm_load_ps((gptrrj));

						__m128 wval1 = _mm_setzero_ps();
						__m128 tval1 = _mm_setzero_ps();
						for(k = 0;  k <= maxk; k ++, ofs++,gofs++,spw++)
						{
							const __m128 bref = _mm_loadu_ps((gptrbj+*gofs));
							const __m128 gref = _mm_loadu_ps((gptrgj+*gofs));
							const __m128 rref = _mm_loadu_ps((gptrrj+*gofs));

							_mm_store_si128((__m128i*)buf,
								_mm_cvtps_epi32(
								_mm_add_ps(
								_mm_add_ps(
								_mm_and_ps(_mm_sub_ps(rval,rref), *(const __m128*)v32f_absmask),
								_mm_and_ps(_mm_sub_ps(gval,gref), *(const __m128*)v32f_absmask)),
								_mm_and_ps(_mm_sub_ps(bval,bref), *(const __m128*)v32f_absmask)
								)
								));
							__m128 vref = _mm_loadu_ps((sptrj+*ofs));
							const __m128 _sw = _mm_set1_ps(*spw);
							__m128 _w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[3]],color_weight[buf[2]],color_weight[buf[1]],color_weight[buf[0]]));

							vref = _mm_mul_ps(_w, vref);
							tval1 = _mm_add_ps(tval1,vref);
							wval1 = _mm_add_ps(wval1,_w);
						}
						tval1 = _mm_div_ps(tval1,wval1);
						_mm_stream_ps((dptr+j), tval1);
					}
				}
#endif
				for(; j < size.width; j++)
				{
					const float* sptrj = sptr+j;
					const float* gptrrj = gptrr+j;
					const float* gptrgj = gptrg+j;
					const float* gptrbj = gptrb+j;

					const float r0 = gptrrj[0];
					const float g0 = gptrgj[0];
					const float b0 = gptrbj[0];

					float sum=0.0f;
					float wsum=0.0f;
					for(k=0 ; k < maxk; k++ )
					{
						const float r = gptrrj[space_guide_ofs[k]], g = gptrgj[space_guide_ofs[k]], b = gptrbj[space_guide_ofs[k]];
						float w = space_weight[k]*color_weight[cvRound(std::abs(b - b0) +std::abs(g - g0) + std::abs(r - r0))];
						sum += sptrj[space_ofs[k]]*w;
						wsum += w;
					}
					dptr[j] = sum/wsum;
				}
			}
		}
		else if(cn == 3 && cng == 3)
		{
			assert( cng == 3 );// color
			int CV_DECL_ALIGNED(16) buf[4];

			const int sstep = 3*temp->cols;
			const int gstep = 3*guide->cols;
			const int dstep = 3*dest->cols;

			float* sptrr =  (float*)temp->ptr<float>(3*radiusV+3*range.start  ) + 4 * (radiusH/4 + 1);
			float* sptrg =  (float*)temp->ptr<float>(3*radiusV+3*range.start+1) + 4 * (radiusH/4 + 1);
			float* sptrb =  (float*)temp->ptr<float>(3*radiusV+3*range.start+2) + 4 * (radiusH/4 + 1);
			float* gptrr = (float*)guide->ptr<float>(3*radiusV+3*range.start  ) + 4 * (radiusH/4 + 1);
			float* gptrg = (float*)guide->ptr<float>(3*radiusV+3*range.start+1) + 4 * (radiusH/4 + 1);
			float* gptrb = (float*)guide->ptr<float>(3*radiusV+3*range.start+2) + 4 * (radiusH/4 + 1);
			float* dptr = dest->ptr<float>(range.start);
			for(i = range.start; i != range.end; i++,gptrr+=gstep,gptrg+=gstep,gptrb+=gstep, sptrr+=sstep,sptrg+=sstep,sptrb+=sstep, dptr+=dstep )
			{	
				j=0;
#if CV_SSE4_1
				if( haveSSE4 )
				{
					for(; j < size.width; j+=4)//4 pixel unit
					{
						int* ofs = &space_ofs[0];
						int* gofs = &space_guide_ofs[0];
						float* spw = space_weight;
						const float* sptrrj = sptrr+j;
						const float* sptrgj = sptrg+j;
						const float* sptrbj = sptrb+j;
						const float* gptrrj = gptrr+j;
						const float* gptrgj = gptrg+j;
						const float* gptrbj = gptrb+j;
						const __m128 bval = _mm_load_ps((gptrbj));
						const __m128 gval = _mm_load_ps((gptrgj));
						const __m128 rval = _mm_load_ps((gptrrj));

						__m128 wval1 = _mm_setzero_ps();
						__m128 rval1 = _mm_setzero_ps();
						__m128 gval1 = _mm_setzero_ps();
						__m128 bval1 = _mm_setzero_ps();

						for(k = 0;  k <= maxk; k ++, ofs++,gofs++,spw++)
						{
							__m128 bref = _mm_loadu_ps((gptrbj+*gofs));
							__m128 gref = _mm_loadu_ps((gptrgj+*gofs));
							__m128 rref = _mm_loadu_ps((gptrrj+*gofs));

							_mm_store_si128((__m128i*)buf,
								_mm_cvtps_epi32(
								_mm_add_ps(
								_mm_add_ps(
								_mm_and_ps(_mm_sub_ps(rval,rref), *(const __m128*)v32f_absmask),
								_mm_and_ps(_mm_sub_ps(gval,gref), *(const __m128*)v32f_absmask)),
								_mm_and_ps(_mm_sub_ps(bval,bref), *(const __m128*)v32f_absmask)
								)
								));

							rref = _mm_loadu_ps((sptrbj+*ofs));
							gref = _mm_loadu_ps((sptrgj+*ofs));
							bref = _mm_loadu_ps((sptrrj+*ofs));

							const __m128 _sw = _mm_set1_ps(*spw);
							__m128 _w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[3]],color_weight[buf[2]],color_weight[buf[1]],color_weight[buf[0]]));

							rref = _mm_mul_ps(_w, rref);
							gref = _mm_mul_ps(_w, gref);
							bref = _mm_mul_ps(_w, bref);

							rval1 = _mm_add_ps(rval1,rref);
							gval1 = _mm_add_ps(gval1,gref);
							bval1 = _mm_add_ps(bval1,bref);
							wval1 = _mm_add_ps(wval1,_w);
						}
						rval1 = _mm_div_ps(rval1,wval1);
						gval1 = _mm_div_ps(gval1,wval1);
						bval1 = _mm_div_ps(bval1,wval1);

						__m128 a = _mm_shuffle_ps(rval1,rval1,_MM_SHUFFLE(3,0,1,2));
						__m128 b = _mm_shuffle_ps(bval1,bval1,_MM_SHUFFLE(1,2,3,0));
						__m128 c = _mm_shuffle_ps(gval1,gval1,_MM_SHUFFLE(2,3,0,1));
						float* dptrc = dptr+3*j;
						_mm_stream_ps((dptrc),_mm_blend_ps(_mm_blend_ps(b,a,4),c,2));
						_mm_stream_ps((dptrc+4),_mm_blend_ps(_mm_blend_ps(c,b,4),a,2));
						_mm_stream_ps((dptrc+8),_mm_blend_ps(_mm_blend_ps(a,c,4),b,2));
					}
				}
#endif
				for(; j < size.width; j++)
				{
					const float* sptrrj = sptrr+j;
					const float* sptrgj = sptrg+j;
					const float* sptrbj = sptrb+j;
					const float* gptrrj = gptrr+j;
					const float* gptrgj = gptrg+j;
					const float* gptrbj = gptrb+j;

					const float r0 = gptrrj[0];
					const float g0 = gptrgj[0];
					const float b0 = gptrbj[0];

					float sum_r=0.0f,sum_b=0.0f,sum_g=0.0f;
					float wsum=0.0f;
					for(k=0 ; k < maxk; k++ )
					{
						const float r = gptrrj[space_guide_ofs[k]], g = gptrgj[space_guide_ofs[k]], b = gptrbj[space_guide_ofs[k]];
						float w = space_weight[k]*color_weight[cvRound(std::abs(b - b0) +std::abs(g - g0) + std::abs(r - r0))];
						sum_b += sptrrj[space_ofs[k]]*w;
						sum_g += sptrgj[space_ofs[k]]*w;
						sum_r += sptrbj[space_ofs[k]]*w;
						wsum += w;
					}
					wsum = 1.f/wsum;
					dptr[3*j  ] = sum_b*wsum;
					dptr[3*j+1] = sum_g*wsum;
					dptr[3*j+2] = sum_r*wsum;
				}
			}

		}
		else if(cn == 3 && cng == 1)
		{
			int CV_DECL_ALIGNED(16) buf[4];

			const int sstep = 3*temp->cols;
			const int gstep =   guide->cols;
			const int dstep = 3*dest->cols;

			float* sptrr = (float*)temp->ptr<float>(3*radiusV+3*range.start  ) + 4 * (radiusH/4 + 1);
			float* sptrg = (float*)temp->ptr<float>(3*radiusV+3*range.start+1) + 4 * (radiusH/4 + 1);
			float* sptrb = (float*)temp->ptr<float>(3*radiusV+3*range.start+2) + 4 * (radiusH/4 + 1);
			float* gptr = (float*)guide->ptr<float>(  radiusV+  range.start) + 4 * (radiusH/4 + 1);
			float* dptr = dest->ptr<float>(range.start);
			for(i = range.start; i != range.end; i++,gptr+=gstep, sptrr+=sstep,sptrg+=sstep,sptrb+=sstep, dptr+=dstep )
			{	
				j=0;
#if CV_SSE4_1
				if( haveSSE4 )
				{
					for(; j < size.width; j+=4)//4 pixel unit
					{
						int* ofs = &space_ofs[0];
						int* gofs = &space_guide_ofs[0];
						float* spw = space_weight;
						const float* sptrrj = sptrr+j;
						const float* sptrgj = sptrg+j;
						const float* sptrbj = sptrb+j;
						const float* gptrj = gptr+j;
						const __m128 sval = _mm_load_ps((gptrj));

						__m128 wval1 = _mm_set1_ps(0.0f);
						__m128 rval1 = _mm_set1_ps(0.0f);
						__m128 gval1 = _mm_set1_ps(0.0f);
						__m128 bval1 = _mm_set1_ps(0.0f);
						for(k = 0;  k <= maxk; k ++, ofs++,gofs++,spw++)
						{
							__m128 sref = _mm_loadu_ps((gptrj+*gofs));
							_mm_store_si128((__m128i*)buf,_mm_cvtps_epi32(_mm_and_ps(_mm_sub_ps(sval,sref), *(const __m128*)v32f_absmask)));

							__m128 rref = _mm_loadu_ps((sptrbj+*ofs));
							__m128 gref = _mm_loadu_ps((sptrgj+*ofs));
							__m128 bref = _mm_loadu_ps((sptrrj+*ofs));

							const __m128 _sw = _mm_set1_ps(*spw);
							__m128 _w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[3]],color_weight[buf[2]],color_weight[buf[1]],color_weight[buf[0]]));

							bref = _mm_mul_ps(_w, bref);
							gref = _mm_mul_ps(_w, gref);
							rref = _mm_mul_ps(_w, rref);

							bval1 = _mm_add_ps(bval1,bref);
							gval1 = _mm_add_ps(gval1,gref);
							rval1 = _mm_add_ps(rval1,rref);
							wval1 = _mm_add_ps(wval1,_w);
						}
						rval1 = _mm_div_ps(rval1,wval1);
						gval1 = _mm_div_ps(gval1,wval1);
						bval1 = _mm_div_ps(bval1,wval1);

						__m128 a = _mm_shuffle_ps(rval1,rval1,_MM_SHUFFLE(3,0,1,2));
						__m128 b = _mm_shuffle_ps(bval1,bval1,_MM_SHUFFLE(1,2,3,0));
						__m128 c = _mm_shuffle_ps(gval1,gval1,_MM_SHUFFLE(2,3,0,1));
						float* dptrc = dptr+3*j;
						_mm_stream_ps((dptrc),_mm_blend_ps(_mm_blend_ps(b,a,4),c,2));
						_mm_stream_ps((dptrc+4),_mm_blend_ps(_mm_blend_ps(c,b,4),a,2));
						_mm_stream_ps((dptrc+8),_mm_blend_ps(_mm_blend_ps(a,c,4),b,2));
					}
				}
#endif
				for(; j < size.width; j++)
				{
					const float* sptrrj = sptrr+j;
					const float* sptrgj = sptrg+j;
					const float* sptrbj = sptrb+j;
					const float* gptrj = gptr+j;

					const float r0 = gptrj[0];
					float sum_r=0.0f,sum_b=0.0f,sum_g=0.0f;
					float wsum=0.0f;
					for(k=0 ; k < maxk; k++ )
					{
						const float r = gptrj[space_guide_ofs[k]];
						float w = space_weight[k]*color_weight[cvRound(std::abs(r - r0))];
						sum_b += sptrrj[space_ofs[k]]*w;
						sum_g += sptrgj[space_ofs[k]]*w;
						sum_r += sptrbj[space_ofs[k]]*w;
						wsum += w;
					}
					wsum = 1.f/wsum;
					dptr[3*j  ] = sum_b*wsum;
					dptr[3*j+1] = sum_g*wsum;
					dptr[3*j+2] = sum_r*wsum;
				}
			}
		}
	}
private:
	const Mat *temp;
	Mat *dest;
	const Mat* guide;
	int radiusH,radiusV, maxk, *space_ofs, *space_guide_ofs;
	float *space_weight, *color_weight;
};

class JointBilateralFilter_8u_InvokerSSE4 : public cv::ParallelLoopBody
{
public:
	JointBilateralFilter_8u_InvokerSSE4(Mat& _dest, const Mat& _temp, const Mat& _guide, int _radiusH, int _radiusV, int _maxk,
		int* _space_ofs, int* _space_guide_ofs, float *_space_weight, float *_color_weight) :
	temp(&_temp), dest(&_dest), guide(&_guide), radiusH(_radiusH), radiusV(_radiusV),
		maxk(_maxk), space_ofs(_space_ofs), space_guide_ofs(_space_guide_ofs), space_weight(_space_weight), color_weight(_color_weight)
	{
	}

	virtual void operator() (const Range& range) const
	{
		int i, j, cn = dest->channels(), k;
		int cng = (guide->rows-2*radiusV) / dest->rows;
		Size size = dest->size();
#if CV_SSE4_1
		bool haveSSE4 = checkHardwareSupport(CV_CPU_SSE4_1);
#endif
		if( cn == 1 && cng ==1)
		{
			uchar CV_DECL_ALIGNED(16) buf[16];

			uchar* sptr = (uchar*)temp->ptr(range.start+radiusV) + 16 * (radiusH/16 + 1);
			uchar* gptr = (uchar*)guide->ptr(range.start+radiusV) + 16 * (radiusH/16 + 1);
			uchar* dptr = dest->ptr(range.start);
			const int sstep = temp->cols;
			const int gstep = guide->cols;
			const int dstep = dest->cols;
			for(i = range.start; i != range.end; i++,dptr+=dstep,sptr+=sstep,gptr+=gstep )
			{
				j=0;
#if CV_SSE4_1
				if( haveSSE4 )
				{
					for(; j < size.width; j+=16)//16 pixel unit
					{
						int* ofs = &space_ofs[0];
						int* gofs = &space_guide_ofs[0];
						float* spw = space_weight;
						const uchar* sptrj = sptr+j;
						const uchar* gptrj = gptr+j;
						const __m128i sval = _mm_load_si128((__m128i*)(gptrj));
						
						__m128 wval1 = _mm_setzero_ps();
						__m128 wval2 = _mm_setzero_ps();
						__m128 wval3 = _mm_setzero_ps();
						__m128 wval4 = _mm_setzero_ps();
						__m128 tval1 = _mm_setzero_ps();
						__m128 tval2 = _mm_setzero_ps();
						__m128 tval3 = _mm_setzero_ps();
						__m128 tval4 = _mm_setzero_ps();

						const __m128i zero = _mm_setzero_si128();
						for(k = 0;  k <= maxk; k ++, ofs++,gofs++,spw++)
						{
							__m128i sref = _mm_loadu_si128((__m128i*)(gptrj+*gofs));
							_mm_store_si128((__m128i*)buf,_mm_add_epi8(_mm_subs_epu8(sval,sref),_mm_subs_epu8(sref,sval)));

							__m128i vref = _mm_loadu_si128((__m128i*)(sptrj+*ofs));

							__m128i m1 = _mm_unpacklo_epi8(vref,zero);
							__m128i m2 = _mm_unpackhi_epi16(m1,zero);
							m1 = _mm_unpacklo_epi16(m1,zero);

							const __m128 _sw = _mm_set1_ps(*spw);

							__m128 _w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[3]],color_weight[buf[2]],color_weight[buf[1]],color_weight[buf[0]]));
							__m128 _valF = _mm_cvtepi32_ps(m1);
							_valF = _mm_mul_ps(_w, _valF);
							tval1 = _mm_add_ps(tval1,_valF);
							wval1 = _mm_add_ps(wval1,_w);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[7]],color_weight[buf[6]],color_weight[buf[5]],color_weight[buf[4]]));
							_valF =_mm_cvtepi32_ps(m2);
							_valF = _mm_mul_ps(_w, _valF);
							tval2 = _mm_add_ps(tval2,_valF);
							wval2 = _mm_add_ps(wval2,_w);

							m1 = _mm_unpackhi_epi8(vref,zero);
							m2 = _mm_unpackhi_epi16(m1,zero);
							m1 = _mm_unpacklo_epi16(m1,zero);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[11]],color_weight[buf[10]],color_weight[buf[9]],color_weight[buf[8]]));
							_valF =_mm_cvtepi32_ps(m1);
							_valF = _mm_mul_ps(_w, _valF);
							wval3 = _mm_add_ps(wval3,_w);
							tval3 = _mm_add_ps(tval3,_valF);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[15]],color_weight[buf[14]],color_weight[buf[13]],color_weight[buf[12]]));
							_valF =_mm_cvtepi32_ps(m2);
							_valF = _mm_mul_ps(_w, _valF);
							wval4 = _mm_add_ps(wval4,_w);
							tval4 = _mm_add_ps(tval4,_valF);
						}
						tval1 = _mm_div_ps(tval1,wval1);
						tval2 = _mm_div_ps(tval2,wval2);
						tval3 = _mm_div_ps(tval3,wval3);
						tval4 = _mm_div_ps(tval4,wval4);
						_mm_stream_si128((__m128i*)(dptr+j), _mm_packus_epi16(_mm_packs_epi32( _mm_cvtps_epi32(tval1), _mm_cvtps_epi32(tval2)) , _mm_packs_epi32( _mm_cvtps_epi32(tval3), _mm_cvtps_epi32(tval4))));
					}
				}
#endif
				for(; j < size.width; j++)
				{
					const uchar val0 = gptr[j];
					float sum=0.0f;
					float wsum=0.0f;
					for(k=0 ; k < maxk; k++ )
					{
						int gval = gptr[j + space_guide_ofs[k]];
						int val = sptr[j + space_ofs[k]];
						float w = space_weight[k]*color_weight[std::abs(gval - val0)];
						sum += val*w;
						wsum += w;
					}
					//overflow is not possible here => there is no need to use CV_CAST_8U
					dptr[j] = (uchar)cvRound(sum/wsum);
				}
			}
		}
		else if(cn == 1 && cng == 3)
		{
			assert( cng == 3 );// color
			short CV_DECL_ALIGNED(16) buf[16];

			const int sstep = temp->cols;
			const int gstep = 3*guide->cols;
			const int dstep = dest->cols;
			uchar* sptr = (uchar*)temp->ptr(range.start+radiusV) + 16 * (radiusH/16 + 1);
			uchar* gptrr = (uchar*)guide->ptr(3*radiusV+3*range.start  ) + 16 * (radiusH/16 + 1);
			uchar* gptrg = (uchar*)guide->ptr(3*radiusV+3*range.start+1) + 16 * (radiusH/16 + 1);
			uchar* gptrb = (uchar*)guide->ptr(3*radiusV+3*range.start+2) + 16 * (radiusH/16 + 1);

			uchar* dptr = dest->ptr(range.start);
			for(i = range.start; i != range.end; i++,gptrr+=gstep,gptrg+=gstep,gptrb+=gstep, sptr+=sstep, dptr+=dstep )
			{	
				j=0;
#if CV_SSE4_1
				if( haveSSE4 )
				{
					for(; j < size.width; j+=16)//16 pixel unit
					{
						__m128i m1,m2,n1,n2;
						__m128 _valF, _w;
						int* ofs = &space_ofs[0];
						int* gofs = &space_guide_ofs[0];
						float* spw = space_weight;
						const uchar* sptrj = sptr+j;
						const uchar* gptrrj = gptrr+j;
						const uchar* gptrgj = gptrg+j;
						const uchar* gptrbj = gptrb+j;
						const __m128i bval = _mm_load_si128((__m128i*)(gptrbj));
						const __m128i gval = _mm_load_si128((__m128i*)(gptrgj));
						const __m128i rval = _mm_load_si128((__m128i*)(gptrrj));

						__m128 wval1 = _mm_setzero_ps();
						__m128 wval2 = _mm_setzero_ps();
						__m128 wval3 = _mm_setzero_ps();
						__m128 wval4 = _mm_setzero_ps();
						__m128 tval1 = _mm_setzero_ps();
						__m128 tval2 = _mm_setzero_ps();
						__m128 tval3 = _mm_setzero_ps();
						__m128 tval4 = _mm_setzero_ps();

						const __m128i zero = _mm_setzero_si128();

						for(k = 0;  k <= maxk; k ++, ofs++,gofs++,spw++)
						{
							const __m128i bref = _mm_loadu_si128((__m128i*)(gptrbj+*gofs));
							const __m128i gref = _mm_loadu_si128((__m128i*)(gptrgj+*gofs));
							const __m128i rref = _mm_loadu_si128((__m128i*)(gptrrj+*gofs));

							m1 = _mm_add_epi8(_mm_subs_epu8(rval,rref),_mm_subs_epu8(rref,rval));
							m2 = _mm_unpackhi_epi8(m1,zero);
							m1 = _mm_unpacklo_epi8(m1,zero);

							n1 = _mm_add_epi8(_mm_subs_epu8(gval,gref),_mm_subs_epu8(gref,gval));
							n2 = _mm_unpackhi_epi8(n1,zero);
							n1 = _mm_unpacklo_epi8(n1,zero);

							m1 = _mm_add_epi16(m1,n1);
							m2 = _mm_add_epi16(m2,n2);

							n1 = _mm_add_epi8(_mm_subs_epu8(bval,bref),_mm_subs_epu8(bref,bval));
							n2 = _mm_unpackhi_epi8(n1,zero);
							n1 = _mm_unpacklo_epi8(n1,zero);

							m1 = _mm_add_epi16(m1,n1);
							m2 = _mm_add_epi16(m2,n2);

							_mm_store_si128((__m128i*)(buf+8),m2);
							_mm_store_si128((__m128i*)buf,m1);

							const __m128i vref = _mm_loadu_si128((__m128i*)(sptrj+*ofs));

							m1 = _mm_unpacklo_epi8(vref,zero);
							m2 = _mm_unpackhi_epi16(m1,zero);
							m1 = _mm_unpacklo_epi16(m1,zero);

							const __m128 _sw = _mm_set1_ps(*spw);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[3]],color_weight[buf[2]],color_weight[buf[1]],color_weight[buf[0]]));
							_valF = _mm_cvtepi32_ps(m1);
							_valF = _mm_mul_ps(_w, _valF);
							tval1 = _mm_add_ps(tval1,_valF);
							wval1 = _mm_add_ps(wval1,_w);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[7]],color_weight[buf[6]],color_weight[buf[5]],color_weight[buf[4]]));
							_valF =_mm_cvtepi32_ps(m2);
							_valF = _mm_mul_ps(_w, _valF);
							tval2 = _mm_add_ps(tval2,_valF);
							wval2 = _mm_add_ps(wval2,_w);


							m1 = _mm_unpackhi_epi8(vref,zero);
							m2 = _mm_unpackhi_epi16(m1,zero);
							m1 = _mm_unpacklo_epi16(m1,zero);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[11]],color_weight[buf[10]],color_weight[buf[9]],color_weight[buf[8]]));
							_valF =_mm_cvtepi32_ps(m1);
							_valF = _mm_mul_ps(_w, _valF);
							tval3 = _mm_add_ps(tval3,_valF);
							wval3 = _mm_add_ps(wval3,_w);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[15]],color_weight[buf[14]],color_weight[buf[13]],color_weight[buf[12]]));
							_valF =_mm_cvtepi32_ps(m2);
							_valF = _mm_mul_ps(_w, _valF);
							tval4 = _mm_add_ps(tval4,_valF);
							wval4 = _mm_add_ps(wval4,_w);
						}
						tval1 = _mm_div_ps(tval1,wval1);
						tval2 = _mm_div_ps(tval2,wval2);
						tval3 = _mm_div_ps(tval3,wval3);
						tval4 = _mm_div_ps(tval4,wval4);
						_mm_store_si128((__m128i*)(dptr+j), _mm_packus_epi16(_mm_packs_epi32( _mm_cvtps_epi32(tval1), _mm_cvtps_epi32(tval2)) , _mm_packs_epi32( _mm_cvtps_epi32(tval3), _mm_cvtps_epi32(tval4))));
					}
				}
#endif
				for(; j < size.width; j++)
				{
					const uchar* sptrj = sptr+j;
					const uchar* gptrrj = gptrr+j;
					const uchar* gptrgj = gptrg+j;
					const uchar* gptrbj = gptrb+j;

					int r0 = gptrrj[0];
					int g0 = gptrgj[0];
					int b0 = gptrbj[0];

					float sum=0.0f;
					float wsum=0.0f;
					for(k=0 ; k < maxk; k++ )
					{
						int r = gptrrj[space_guide_ofs[k]], g = gptrgj[space_guide_ofs[k]], b = gptrbj[space_guide_ofs[k]];
						float w = space_weight[k]*color_weight[std::abs(b - b0) +std::abs(g - g0) + std::abs(r - r0)];
						sum += sptrj[space_ofs[k]]*w;
						wsum += w;
					}
					//overflow is not possible here => there is no need to use CV_CAST_8U
					dptr[j] = (uchar)cvRound(sum/wsum);
				}
			}
		}
		else if(cn == 3 && cng == 3)
		{
			assert( cng == 3 );//color
			short CV_DECL_ALIGNED(16) buf[16];

			const int sstep = 3*temp->cols;
			const int gstep = 3*guide->cols;
			const int dstep = 3*dest->cols;

			uchar* sptrr =  (uchar*)temp->ptr(3*radiusV+3*range.start  ) + 16 * (radiusH/16 + 1);
			uchar* sptrg =  (uchar*)temp->ptr(3*radiusV+3*range.start+1) + 16 * (radiusH/16 + 1);
			uchar* sptrb =  (uchar*)temp->ptr(3*radiusV+3*range.start+2) + 16 * (radiusH/16 + 1);
			uchar* gptrr = (uchar*)guide->ptr(3*radiusV+3*range.start  ) + 16 * (radiusH/16 + 1);
			uchar* gptrg = (uchar*)guide->ptr(3*radiusV+3*range.start+1) + 16 * (radiusH/16 + 1);
			uchar* gptrb = (uchar*)guide->ptr(3*radiusV+3*range.start+2) + 16 * (radiusH/16 + 1);
			uchar* dptr = dest->ptr(range.start);
			for(i = range.start; i != range.end; i++,gptrr+=gstep,gptrg+=gstep,gptrb+=gstep, sptrr+=sstep,sptrg+=sstep,sptrb+=sstep, dptr+=dstep )
			{	
				j=0;
#if CV_SSE4_1
				if( haveSSE4 )
				{
					for(; j < size.width; j+=16)//16 pixel unit
					{
						int* ofs = &space_ofs[0];
						int* gofs = &space_guide_ofs[0];
						float* spw = space_weight;
						const uchar* sptrrj = sptrr+j;
						const uchar* sptrgj = sptrg+j;
						const uchar* sptrbj = sptrb+j;
						const uchar* gptrrj = gptrr+j;
						const uchar* gptrgj = gptrg+j;
						const uchar* gptrbj = gptrb+j;
						const __m128i bval = _mm_load_si128((__m128i*)(gptrbj));
						const __m128i gval = _mm_load_si128((__m128i*)(gptrgj));
						const __m128i rval = _mm_load_si128((__m128i*)(gptrrj));

						__m128 wval1 = _mm_setzero_ps();
						__m128 wval2 = _mm_setzero_ps();
						__m128 wval3 = _mm_setzero_ps();
						__m128 wval4 = _mm_setzero_ps();

						__m128 rval1 = _mm_setzero_ps();
						__m128 rval2 = _mm_setzero_ps();
						__m128 rval3 = _mm_setzero_ps();
						__m128 rval4 = _mm_setzero_ps();

						__m128 gval1 = _mm_setzero_ps();
						__m128 gval2 = _mm_setzero_ps();
						__m128 gval3 = _mm_setzero_ps();
						__m128 gval4 = _mm_setzero_ps();

						__m128 bval1 = _mm_setzero_ps();
						__m128 bval2 = _mm_setzero_ps();
						__m128 bval3 = _mm_setzero_ps();
						__m128 bval4 = _mm_setzero_ps();

						const __m128i zero = _mm_setzero_si128();
						for(k = 0;  k <= maxk; k ++, ofs++,gofs++,spw++)
						{
							__m128i bref = _mm_loadu_si128((__m128i*)(gptrbj+*gofs));
							__m128i gref = _mm_loadu_si128((__m128i*)(gptrgj+*gofs));
							__m128i rref = _mm_loadu_si128((__m128i*)(gptrrj+*gofs));

							__m128i r1 = _mm_add_epi8(_mm_subs_epu8(rval,rref),_mm_subs_epu8(rref,rval));
							__m128i r2 = _mm_unpackhi_epi8(r1,zero);
							r1 = _mm_unpacklo_epi8(r1,zero);

							__m128i g1 = _mm_add_epi8(_mm_subs_epu8(gval,gref),_mm_subs_epu8(gref,gval));
							__m128i g2 = _mm_unpackhi_epi8(g1,zero);
							g1 = _mm_unpacklo_epi8(g1,zero);

							r1 = _mm_add_epi16(r1,g1);
							r2 = _mm_add_epi16(r2,g2);

							__m128i b1 = _mm_add_epi8(_mm_subs_epu8(bval,bref),_mm_subs_epu8(bref,bval));
							__m128i b2 = _mm_unpackhi_epi8(b1,zero);
							b1 = _mm_unpacklo_epi8(b1,zero);

							r1 = _mm_add_epi16(r1,b1);
							r2 = _mm_add_epi16(r2,b2);

							_mm_store_si128((__m128i*)(buf+8),r2);
							_mm_store_si128((__m128i*)buf,r1);

							bref = _mm_loadu_si128((__m128i*)(sptrbj+*ofs));
							gref = _mm_loadu_si128((__m128i*)(sptrgj+*ofs));
							rref = _mm_loadu_si128((__m128i*)(sptrrj+*ofs));

							r1 = _mm_unpacklo_epi8(rref,zero);
							r2 = _mm_unpackhi_epi16(r1,zero);
							r1 = _mm_unpacklo_epi16(r1,zero);
							g1 = _mm_unpacklo_epi8(gref,zero);
							g2 = _mm_unpackhi_epi16(g1,zero);
							g1 = _mm_unpacklo_epi16(g1,zero);
							b1 = _mm_unpacklo_epi8(bref,zero);
							b2 = _mm_unpackhi_epi16(b1,zero);
							b1 = _mm_unpacklo_epi16(b1,zero);

							const __m128 _sw = _mm_set1_ps(*spw);
							__m128 _w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[3]],color_weight[buf[2]],color_weight[buf[1]],color_weight[buf[0]]));

							__m128 _valr = _mm_cvtepi32_ps(r1);
							__m128 _valg = _mm_cvtepi32_ps(g1);
							__m128 _valb = _mm_cvtepi32_ps(b1);

							_valr = _mm_mul_ps(_w, _valr);
							_valg = _mm_mul_ps(_w, _valg);
							_valb = _mm_mul_ps(_w, _valb);

							rval1 = _mm_add_ps(rval1,_valr);
							gval1 = _mm_add_ps(gval1,_valg);
							bval1 = _mm_add_ps(bval1,_valb);
							wval1 = _mm_add_ps(wval1,_w);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[7]],color_weight[buf[6]],color_weight[buf[5]],color_weight[buf[4]]));

							_valr =_mm_cvtepi32_ps(r2);
							_valg =_mm_cvtepi32_ps(g2);
							_valb =_mm_cvtepi32_ps(b2);

							_valr = _mm_mul_ps(_w, _valr);
							_valg = _mm_mul_ps(_w, _valg);
							_valb = _mm_mul_ps(_w, _valb);

							rval2 = _mm_add_ps(rval2,_valr);
							gval2 = _mm_add_ps(gval2,_valg);
							bval2 = _mm_add_ps(bval2,_valb);
							wval2 = _mm_add_ps(wval2,_w);

							r1 = _mm_unpackhi_epi8(rref,zero);
							r2 = _mm_unpackhi_epi16(r1,zero);
							r1 = _mm_unpacklo_epi16(r1,zero);

							g1 = _mm_unpackhi_epi8(gref,zero);
							g2 = _mm_unpackhi_epi16(g1,zero);
							g1 = _mm_unpacklo_epi16(g1,zero);

							b1 = _mm_unpackhi_epi8(bref,zero);
							b2 = _mm_unpackhi_epi16(b1,zero);
							b1 = _mm_unpacklo_epi16(b1,zero);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[11]],color_weight[buf[10]],color_weight[buf[9]],color_weight[buf[8]]));

							_valr =_mm_cvtepi32_ps(r1);
							_valg =_mm_cvtepi32_ps(g1);
							_valb =_mm_cvtepi32_ps(b1);

							_valr = _mm_mul_ps(_w, _valr);
							_valg = _mm_mul_ps(_w, _valg);
							_valb = _mm_mul_ps(_w, _valb);

							rval3 = _mm_add_ps(rval3,_valr);
							gval3 = _mm_add_ps(gval3,_valg);
							bval3 = _mm_add_ps(bval3,_valb);
							wval3 = _mm_add_ps(wval3,_w);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[15]],color_weight[buf[14]],color_weight[buf[13]],color_weight[buf[12]]));

							_valr =_mm_cvtepi32_ps(r2);
							_valg =_mm_cvtepi32_ps(g2);
							_valb =_mm_cvtepi32_ps(b2);

							_valr = _mm_mul_ps(_w, _valr);
							_valg = _mm_mul_ps(_w, _valg);
							_valb = _mm_mul_ps(_w, _valb);

							rval4 = _mm_add_ps(rval4,_valr);
							gval4 = _mm_add_ps(gval4,_valg);
							bval4 = _mm_add_ps(bval4,_valb);	
							wval4 = _mm_add_ps(wval4,_w);
						}
						rval1 = _mm_div_ps(rval1,wval1);
						rval2 = _mm_div_ps(rval2,wval2);
						rval3 = _mm_div_ps(rval3,wval3);
						rval4 = _mm_div_ps(rval4,wval4);
						__m128i a = _mm_packus_epi16(_mm_packs_epi32( _mm_cvtps_epi32(rval1), _mm_cvtps_epi32(rval2)) , _mm_packs_epi32( _mm_cvtps_epi32(rval3), _mm_cvtps_epi32(rval4)));
						gval1 = _mm_div_ps(gval1,wval1);
						gval2 = _mm_div_ps(gval2,wval2);
						gval3 = _mm_div_ps(gval3,wval3);
						gval4 = _mm_div_ps(gval4,wval4);
						__m128i b = _mm_packus_epi16(_mm_packs_epi32( _mm_cvtps_epi32(gval1), _mm_cvtps_epi32(gval2)) , _mm_packs_epi32( _mm_cvtps_epi32(gval3), _mm_cvtps_epi32(gval4)));
						bval1 = _mm_div_ps(bval1,wval1);
						bval2 = _mm_div_ps(bval2,wval2);
						bval3 = _mm_div_ps(bval3,wval3);
						bval4 = _mm_div_ps(bval4,wval4);
						__m128i c = _mm_packus_epi16(_mm_packs_epi32( _mm_cvtps_epi32(bval1), _mm_cvtps_epi32(bval2)) , _mm_packs_epi32( _mm_cvtps_epi32(bval3), _mm_cvtps_epi32(bval4)));

						//sse4///
						uchar* dptrc = dptr+3*j;
						const __m128i mask1 = _mm_setr_epi8(0, 11, 6, 1, 12, 7, 2, 13, 8, 3, 14, 9, 4, 15, 10, 5);
						const __m128i mask2 = _mm_setr_epi8(5, 0, 11, 6, 1, 12, 7, 2, 13, 8, 3, 14, 9, 4, 15, 10);
						const __m128i mask3 = _mm_setr_epi8(10, 5, 0, 11, 6, 1, 12, 7, 2, 13, 8, 3, 14, 9, 4, 15);
						const __m128i bmask1 = _mm_setr_epi8(0,255,255,0,255,255,0,255,255,0,255,255,0,255,255,0);
						const __m128i bmask2 = _mm_setr_epi8(255,255,0,255,255,0,255,255,0,255,255,0,255,255,0,255);

						a = _mm_shuffle_epi8(a,mask1);
						b = _mm_shuffle_epi8(b,mask2);
						c = _mm_shuffle_epi8(c,mask3);
						_mm_stream_si128((__m128i*)(dptrc),_mm_blendv_epi8(c,_mm_blendv_epi8(a,b,bmask1),bmask2));
						_mm_stream_si128((__m128i*)(dptrc+16),_mm_blendv_epi8(b,_mm_blendv_epi8(a,c,bmask2),bmask1));		
						_mm_stream_si128((__m128i*)(dptrc+32),_mm_blendv_epi8(c,_mm_blendv_epi8(b,a,bmask2),bmask1));
					}
				}
#endif
				for(; j < size.width; j++)
				{
					const uchar* sptrrj = sptrr+j;
					const uchar* sptrgj = sptrg+j;
					const uchar* sptrbj = sptrb+j;
					const uchar* gptrrj = gptrr+j;
					const uchar* gptrgj = gptrg+j;
					const uchar* gptrbj = gptrb+j;

					int r0 = gptrrj[0];
					int g0 = gptrgj[0];
					int b0 = gptrbj[0];

					float sum_r=0.0f,sum_b=0.0f,sum_g=0.0f;
					float wsum=0.0f;
					for(k=0 ; k < maxk; k++ )
					{
						int r = gptrrj[space_guide_ofs[k]], g = gptrgj[space_guide_ofs[k]], b = gptrbj[space_guide_ofs[k]];
						float w = space_weight[k]*color_weight[std::abs(b - b0) +std::abs(g - g0) + std::abs(r - r0)];
						sum_b += sptrrj[space_ofs[k]]*w;
						sum_g += sptrgj[space_ofs[k]]*w;
						sum_r += sptrbj[space_ofs[k]]*w;
						wsum += w;
					}
					
					wsum = 1.f/wsum;
					b0 = cvRound(sum_b*wsum);
					g0 = cvRound(sum_g*wsum);
					r0 = cvRound(sum_r*wsum);
					dptr[3*j] = (uchar)b0; dptr[3*j+1] = (uchar)g0; dptr[3*j+2] = (uchar)r0;
				}
			}

		}
		else if(cn == 3 && cng == 1)
		{
			uchar CV_DECL_ALIGNED(16) buf[16];

			const int sstep = 3*temp->cols;
			const int gstep =   guide->cols;
			const int dstep = 3*dest->cols;

			uchar* sptrr = (uchar*)temp->ptr(3*radiusV+3*range.start  ) + 16 * (radiusH/16 + 1);
			uchar* sptrg = (uchar*)temp->ptr(3*radiusV+3*range.start+1) + 16 * (radiusH/16 + 1);
			uchar* sptrb = (uchar*)temp->ptr(3*radiusV+3*range.start+2) + 16 * (radiusH/16 + 1);
			uchar* gptr = (uchar*)guide->ptr(  radiusV+  range.start) + 16 * (radiusH/16 + 1);
			uchar* dptr = dest->ptr(range.start);
			for(i = range.start; i != range.end; i++,gptr+=gstep, sptrr+=sstep,sptrg+=sstep,sptrb+=sstep, dptr+=dstep )
			{	
				j=0;
#if CV_SSE4_1
				if( haveSSE4 )
				{
					for(; j < size.width; j+=16)//16 pixel unit
					{
						int* ofs = &space_ofs[0];
						int* gofs = &space_guide_ofs[0];
						float* spw = space_weight;
						const uchar* sptrrj = sptrr+j;
						const uchar* sptrgj = sptrg+j;
						const uchar* sptrbj = sptrb+j;
						const uchar* gptrj = gptr+j;
						const __m128i sval = _mm_load_si128((__m128i*)(gptrj));

						__m128 wval1 = _mm_set1_ps(0.0f);
						__m128 rval1 = _mm_set1_ps(0.0f);
						__m128 gval1 = _mm_set1_ps(0.0f);
						__m128 bval1 = _mm_set1_ps(0.0f);

						__m128 wval2 = _mm_set1_ps(0.0f);
						__m128 rval2 = _mm_set1_ps(0.0f);
						__m128 gval2 = _mm_set1_ps(0.0f);
						__m128 bval2 = _mm_set1_ps(0.0f);

						__m128 wval3 = _mm_set1_ps(0.0f);
						__m128 rval3 = _mm_set1_ps(0.0f);
						__m128 gval3 = _mm_set1_ps(0.0f);
						__m128 bval3 = _mm_set1_ps(0.0f);

						__m128 wval4 = _mm_set1_ps(0.0f);
						__m128 rval4 = _mm_set1_ps(0.0f);
						__m128 gval4 = _mm_set1_ps(0.0f);
						__m128 bval4 = _mm_set1_ps(0.0f);

						const __m128i zero = _mm_setzero_si128();
						for(k = 0;  k <= maxk; k ++, ofs++,gofs++,spw++)
						{
							__m128i sref = _mm_loadu_si128((__m128i*)(gptrj+*gofs));
							_mm_store_si128((__m128i*)buf,_mm_add_epi8(_mm_subs_epu8(sval,sref),_mm_subs_epu8(sref,sval)));

							__m128i bref = _mm_loadu_si128((__m128i*)(sptrbj+*ofs));
							__m128i gref = _mm_loadu_si128((__m128i*)(sptrgj+*ofs));
							__m128i rref = _mm_loadu_si128((__m128i*)(sptrrj+*ofs));

							__m128i r1 = _mm_unpacklo_epi8(rref,zero);
							__m128i r2 = _mm_unpackhi_epi16(r1,zero);
							r1 = _mm_unpacklo_epi16(r1,zero);
							__m128i g1 = _mm_unpacklo_epi8(gref,zero);
							__m128i g2 = _mm_unpackhi_epi16(g1,zero);
							g1 = _mm_unpacklo_epi16(g1,zero);
							__m128i b1 = _mm_unpacklo_epi8(bref,zero);
							__m128i b2 = _mm_unpackhi_epi16(b1,zero);
							b1 = _mm_unpacklo_epi16(b1,zero);

							const __m128 _sw = _mm_set1_ps(*spw);
							__m128 _w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[3]],color_weight[buf[2]],color_weight[buf[1]],color_weight[buf[0]]));

							__m128 _valr = _mm_cvtepi32_ps(r1);
							__m128 _valg = _mm_cvtepi32_ps(g1);
							__m128 _valb = _mm_cvtepi32_ps(b1);

							_valr = _mm_mul_ps(_w, _valr);
							_valg = _mm_mul_ps(_w, _valg);
							_valb = _mm_mul_ps(_w, _valb);

							rval1 = _mm_add_ps(rval1,_valr);
							gval1 = _mm_add_ps(gval1,_valg);
							bval1 = _mm_add_ps(bval1,_valb);
							wval1 = _mm_add_ps(wval1,_w);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[7]],color_weight[buf[6]],color_weight[buf[5]],color_weight[buf[4]]));

							_valr =_mm_cvtepi32_ps(r2);
							_valg =_mm_cvtepi32_ps(g2);
							_valb =_mm_cvtepi32_ps(b2);

							_valr = _mm_mul_ps(_w, _valr);//�l�Əd�ݑS�̂Ƃ̐�
							_valg = _mm_mul_ps(_w, _valg);//�l�Əd�ݑS�̂Ƃ̐�
							_valb = _mm_mul_ps(_w, _valb);//�l�Əd�ݑS�̂Ƃ̐�

							rval2 = _mm_add_ps(rval2,_valr);
							gval2 = _mm_add_ps(gval2,_valg);
							bval2 = _mm_add_ps(bval2,_valb);
							wval2 = _mm_add_ps(wval2,_w);

							r1 = _mm_unpackhi_epi8(rref,zero);
							r2 = _mm_unpackhi_epi16(r1,zero);
							r1 = _mm_unpacklo_epi16(r1,zero);

							g1 = _mm_unpackhi_epi8(gref,zero);
							g2 = _mm_unpackhi_epi16(g1,zero);
							g1 = _mm_unpacklo_epi16(g1,zero);

							b1 = _mm_unpackhi_epi8(bref,zero);
							b2 = _mm_unpackhi_epi16(b1,zero);
							b1 = _mm_unpacklo_epi16(b1,zero);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[11]],color_weight[buf[10]],color_weight[buf[9]],color_weight[buf[8]]));

							_valr =_mm_cvtepi32_ps(r1);
							_valg =_mm_cvtepi32_ps(g1);
							_valb =_mm_cvtepi32_ps(b1);

							_valr = _mm_mul_ps(_w, _valr);
							_valg = _mm_mul_ps(_w, _valg);
							_valb = _mm_mul_ps(_w, _valb);

							wval3 = _mm_add_ps(wval3,_w);
							rval3 = _mm_add_ps(rval3,_valr);
							gval3 = _mm_add_ps(gval3,_valg);
							bval3 = _mm_add_ps(bval3,_valb);

							_w = _mm_mul_ps(_sw,_mm_set_ps(color_weight[buf[15]],color_weight[buf[14]],color_weight[buf[13]],color_weight[buf[12]]));

							_valr =_mm_cvtepi32_ps(r2);
							_valg =_mm_cvtepi32_ps(g2);
							_valb =_mm_cvtepi32_ps(b2);

							_valr = _mm_mul_ps(_w, _valr);
							_valg = _mm_mul_ps(_w, _valg);
							_valb = _mm_mul_ps(_w, _valb);

							wval4 = _mm_add_ps(wval4,_w);
							rval4 = _mm_add_ps(rval4,_valr);
							gval4 = _mm_add_ps(gval4,_valg);
							bval4 = _mm_add_ps(bval4,_valb);	
						}
						rval1 = _mm_div_ps(rval1,wval1);
						rval2 = _mm_div_ps(rval2,wval2);
						rval3 = _mm_div_ps(rval3,wval3);
						rval4 = _mm_div_ps(rval4,wval4);
						__m128i a = _mm_packus_epi16(_mm_packs_epi32( _mm_cvtps_epi32(rval1), _mm_cvtps_epi32(rval2)) , _mm_packs_epi32( _mm_cvtps_epi32(rval3), _mm_cvtps_epi32(rval4)));
						gval1 = _mm_div_ps(gval1,wval1);
						gval2 = _mm_div_ps(gval2,wval2);
						gval3 = _mm_div_ps(gval3,wval3);
						gval4 = _mm_div_ps(gval4,wval4);
						__m128i b = _mm_packus_epi16(_mm_packs_epi32( _mm_cvtps_epi32(gval1), _mm_cvtps_epi32(gval2)) , _mm_packs_epi32( _mm_cvtps_epi32(gval3), _mm_cvtps_epi32(gval4)));
						bval1 = _mm_div_ps(bval1,wval1);
						bval2 = _mm_div_ps(bval2,wval2);
						bval3 = _mm_div_ps(bval3,wval3);
						bval4 = _mm_div_ps(bval4,wval4);
						__m128i c = _mm_packus_epi16(_mm_packs_epi32( _mm_cvtps_epi32(bval1), _mm_cvtps_epi32(bval2)) , _mm_packs_epi32( _mm_cvtps_epi32(bval3), _mm_cvtps_epi32(bval4)));

						//sse4///
						uchar* dptrc = dptr+3*j;
						const __m128i mask1 = _mm_setr_epi8(0, 11, 6, 1, 12, 7, 2, 13, 8, 3, 14, 9, 4, 15, 10, 5);
						const __m128i mask2 = _mm_setr_epi8(5, 0, 11, 6, 1, 12, 7, 2, 13, 8, 3, 14, 9, 4, 15, 10);
						const __m128i mask3 = _mm_setr_epi8(10, 5, 0, 11, 6, 1, 12, 7, 2, 13, 8, 3, 14, 9, 4, 15);

						const __m128i bmask1 = _mm_setr_epi8
							(0,255,255,0,255,255,0,255,255,0,255,255,0,255,255,0);

						const __m128i bmask2 = _mm_setr_epi8
							(255,255,0,255,255,0,255,255,0,255,255,0,255,255,0,255);

						a = _mm_shuffle_epi8(a,mask1);
						b = _mm_shuffle_epi8(b,mask2);
						c = _mm_shuffle_epi8(c,mask3);

						_mm_stream_si128((__m128i*)(dptrc),_mm_blendv_epi8(c,_mm_blendv_epi8(a,b,bmask1),bmask2));
						_mm_stream_si128((__m128i*)(dptrc+16),_mm_blendv_epi8(b,_mm_blendv_epi8(a,c,bmask2),bmask1));		
						_mm_stream_si128((__m128i*)(dptrc+32),_mm_blendv_epi8(c,_mm_blendv_epi8(b,a,bmask2),bmask1));
					}
				}
#endif
				for(; j < size.width; j++)
				{
					const uchar* sptrrj = sptrr+j;
					const uchar* sptrgj = sptrg+j;
					const uchar* sptrbj = sptrb+j;
					const uchar* gptrj = gptr+j;

					int r0 = gptrj[0];

					float sum_r=0.0f,sum_b=0.0f,sum_g=0.0f;
					float wsum=0.0f;
					for(k=0 ; k < maxk; k++ )
					{
						int r = gptrj[space_guide_ofs[k]];
						float w = space_weight[k]*color_weight[std::abs(r - r0)];
						sum_b += sptrrj[space_ofs[k]]*w;
						sum_g += sptrgj[space_ofs[k]]*w;
						sum_r += sptrbj[space_ofs[k]]*w;
						wsum += w;
					}
					wsum = 1.f/wsum;
					int b0 = cvRound(sum_b*wsum);
					int g0 = cvRound(sum_g*wsum);
					r0 = cvRound(sum_r*wsum);
					dptr[3*j] = (uchar)b0; dptr[3*j+1] = (uchar)g0; dptr[3*j+2] = (uchar)r0;
				}
			}
		}
	}
private:
	const Mat *temp;
	Mat *dest;
	const Mat* guide;
	int radiusH,radiusV, maxk, *space_ofs, *space_guide_ofs;
	float *space_weight, *color_weight;
};

void weightedJointBilateralFilter_32f( const Mat& src, Mat& weight, const Mat& guide, Mat& dst, Size kernelSize, double sigma_color, double sigma_space, int borderType )
{
	if(kernelSize.width<=1 && kernelSize.height<=1){ src.copyTo(dst);return;}

	int cn = src.channels();
	int cng = guide.channels();
	int i, j, maxk;
	Size size = src.size();

	CV_Assert( (src.type() == CV_32FC1   || src.type() == CV_32FC3) &&
		(guide.type() == CV_32FC1 || guide.type() == CV_32FC3) &&
		src.type() == dst.type() && src.size() == dst.size());

	if( sigma_color <= 0 )
		sigma_color = 1;
	if( sigma_space <= 0 )
		sigma_space = 1;

	double gauss_color_coeff = -0.5/(sigma_color*sigma_color);
	double gauss_space_coeff = -0.5/(sigma_space*sigma_space);

	int radiusH = kernelSize.width>>1;
	int radiusV = kernelSize.height>>1;

	Mat temp,tempg,tempw;

	int dpad = (4- src.cols%4)%4;
	int spad =  dpad + (4-(2*radiusH)%4)%4;
	if(spad<4) spad +=4;
	int lpad = 4*(radiusH/4+1)-radiusH;
	int rpad = spad-lpad;

	if(cn==1 && cng==1)
	{
		copyMakeBorder( src, temp, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		copyMakeBorder( guide, tempg, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		copyMakeBorder( weight, tempw, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
	}
	else if(cn==1 && cng==3)
	{
		copyMakeBorder( src, temp, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		Mat temp2;
		copyMakeBorder( guide, temp2, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		splitBGRLineInterleave(temp2,tempg);

		copyMakeBorder( weight, tempw, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
	}
	else if(cn==3 && cng==3)
	{
		Mat temp2;
		copyMakeBorder( src, temp2, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		splitBGRLineInterleave(temp2,temp);

		copyMakeBorder( guide, temp2, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		splitBGRLineInterleave(temp2,tempg);

		copyMakeBorder( weight, tempw, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
	}
	else if(cn==3 && cng==1)
	{
		Mat temp2;
		copyMakeBorder( src, temp2, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		splitBGRLineInterleave(temp2,temp);

		copyMakeBorder( guide, tempg, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );

		copyMakeBorder( weight, tempw, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
	}

	double minv,maxv;
	minMaxLoc(guide,&minv,&maxv);
	const int color_range = cvRound(maxv-minv);

	vector<float> _color_weight(cng*color_range);
	vector<float> _space_weight(kernelSize.area()+1);
	vector<int> _space_ofs(kernelSize.area()+1);
	vector<int> _space_w_ofs(kernelSize.area()+1);
	vector<int> _space_guide_ofs(kernelSize.area()+1);
	float* color_weight = &_color_weight[0];
	float* space_weight = &_space_weight[0];
	int* space_ofs = &_space_ofs[0];
	int* space_w_ofs = &_space_w_ofs[0];
	int* space_guide_ofs = &_space_guide_ofs[0];

	// initialize color-related bilateral filter coefficients

	for( i = 0; i < color_range*cng; i++ )
		color_weight[i] = (float)std::exp(i*i*gauss_color_coeff);

	// initialize space-related bilateral filter coefficients
	for( i = -radiusV, maxk = 0; i <= radiusV; i++ )
	{
		j = -radiusH;

		for( ;j <= radiusH; j++ )
		{
			double r = std::sqrt((double)i*i + (double)j*j);
			if( r > max(radiusV,radiusH) )
				continue;
			space_weight[maxk] = (float)std::exp(r*r*gauss_space_coeff);
			space_ofs[maxk] = (int)(i*temp.cols*cn + j);
			space_w_ofs[maxk] = (int)(i*tempw.cols   + j);
			space_guide_ofs[maxk++] = (int)(i*tempg.cols*cng + j);
		}
	}

	Mat dest = Mat::zeros(Size(src.cols+dpad, src.rows),dst.type());
	WeightedJointBilateralFilter_32f_InvokerSSE4 body(dest, temp, tempw,tempg,radiusH,radiusV, maxk, space_ofs, space_w_ofs,space_guide_ofs,space_weight, color_weight);
	parallel_for_(Range(0, size.height), body);
	Mat(dest(Rect(0,0,dst.cols,dst.rows))).copyTo(dst);
}

void weightedJointBilateralFilter_8u( const Mat& src, Mat& weight, const Mat& guide, Mat& dst, Size kernelSize, double sigma_color, double sigma_space, int borderType )
{
	if(kernelSize.width<=1 && kernelSize.height<=1){ src.copyTo(dst);return;}

	int cn = src.channels();
	int cng = guide.channels();
	int i, j, maxk;
	Size size = src.size();

	CV_Assert( (src.type() == CV_8UC1   || src.type() == CV_8UC3) &&
		(guide.type() == CV_8UC1 || guide.type() == CV_8UC3) &&
		src.type() == dst.type() && src.size() == dst.size());

	if( sigma_color <= 0 )
		sigma_color = 1;
	if( sigma_space <= 0 )
		sigma_space = 1;

	double gauss_color_coeff = -0.5/(sigma_color*sigma_color);
	double gauss_space_coeff = -0.5/(sigma_space*sigma_space);

	int radiusH = kernelSize.width>>1;
	int radiusV = kernelSize.height>>1;

	Mat temp,tempg,tempw;

	int dpad = (16- src.cols%16)%16;
	int spad =  dpad + (16-(2*radiusH)%16)%16;
	if(spad<16) spad +=16;
	int lpad = 16*(radiusH/16+1)-radiusH;
	int rpad = spad-lpad;

	if(cn==1 && cng==1)
	{
		copyMakeBorder( src, temp, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		copyMakeBorder( guide, tempg, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		copyMakeBorder( weight, tempw, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
	}
	else if(cn==1 && cng==3)
	{
		copyMakeBorder( src, temp, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		Mat temp2;
		copyMakeBorder( guide, temp2, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		splitBGRLineInterleave(temp2,tempg);

		copyMakeBorder( weight, tempw, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
	}
	else if(cn==3 && cng==3)
	{
		Mat temp2;
		copyMakeBorder( src, temp2, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		splitBGRLineInterleave(temp2,temp);

		copyMakeBorder( guide, temp2, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		splitBGRLineInterleave(temp2,tempg);

		copyMakeBorder( weight, tempw, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
	}
	else if(cn==3 && cng==1)
	{
		Mat temp2;
		copyMakeBorder( src, temp2, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		splitBGRLineInterleave(temp2,temp);

		copyMakeBorder( guide, tempg, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );

		copyMakeBorder( weight, tempw, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
	}
	/*
	double minv,maxv;
	minMaxLoc(guide,&minv,&maxv);
	const int color_range = cvRound(maxv-minv);*/
	const int color_range=256;

	vector<float> _color_weight(cng*color_range);
	vector<float> _space_weight(kernelSize.area()+1);
	vector<int> _space_ofs(kernelSize.area()+1);
	vector<int> _space_w_ofs(kernelSize.area()+1);
	vector<int> _space_guide_ofs(kernelSize.area()+1);
	float* color_weight = &_color_weight[0];
	float* space_weight = &_space_weight[0];
	int* space_ofs = &_space_ofs[0];
	int* space_w_ofs = &_space_w_ofs[0];
	int* space_guide_ofs = &_space_guide_ofs[0];

	// initialize color-related bilateral filter coefficients

	for( i = 0; i < color_range*cng; i++ )
		color_weight[i] = (float)std::exp(i*i*gauss_color_coeff);

	// initialize space-related bilateral filter coefficients
	for( i = -radiusV, maxk = 0; i <= radiusV; i++ )
	{
		j = -radiusH;

		for( ;j <= radiusH; j++ )
		{
			double r = std::sqrt((double)i*i + (double)j*j);
			if( r > max(radiusV,radiusH) )
				continue;
			space_weight[maxk] = (float)std::exp(r*r*gauss_space_coeff);
			space_ofs[maxk] = (int)(i*temp.cols*cn + j);
			space_w_ofs[maxk] = (int)(i*tempw.cols   + j);
			space_guide_ofs[maxk++] = (int)(i*tempg.cols*cng + j);
		}
	}

	Mat dest = Mat::zeros(Size(src.cols+dpad, src.rows),dst.type());
	WeightedJointBilateralFilter_8u_InvokerSSE4 body(dest, temp, tempw,tempg,radiusH,radiusV, maxk, space_ofs, space_w_ofs,space_guide_ofs,space_weight, color_weight);
	parallel_for_(Range(0, size.height), body);
	Mat(dest(Rect(0,0,dst.cols,dst.rows))).copyTo(dst);
}


void jointBilateralFilter_32f( const Mat& src, const Mat& guide, Mat& dst, Size kernelSize , double sigma_color, double sigma_space, int borderType )
{
	if(kernelSize.width==0 || kernelSize.height==0){ src.copyTo(dst);return;}
	int cn = src.channels();
	int cng = guide.channels();
	int i, j, maxk;
	Size size = src.size();

	CV_Assert( (src.type() == CV_32FC1   || src.type() == CV_32FC3) &&
		(guide.type() == CV_32FC1 || guide.type() == CV_32FC3) &&
		src.type() == dst.type() && src.size() == dst.size());

	if( sigma_color <= 0 )
		sigma_color = 1;
	if( sigma_space <= 0 )
		sigma_space = 1;

	double gauss_color_coeff = -0.5/(sigma_color*sigma_color);
	double gauss_space_coeff = -0.5/(sigma_space*sigma_space);

	int radiusH = kernelSize.width>>1;
	int radiusV = kernelSize.height>>1;

	Mat temp,tempg;

	int dpad = (4- src.cols%4)%4;
	int spad =  dpad + (4-(2*radiusH)%4)%4;
	if(spad<4) spad +=4;
	int lpad = 4*(radiusH/4+1)-radiusH;
	int rpad = spad-lpad;
	if(cn==1 && cng==1)
	{
		copyMakeBorder( src, temp, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		copyMakeBorder( guide, tempg, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
	}
	else if(cn==1 && cng==3)
	{
		copyMakeBorder( src, temp, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		Mat temp2;
		copyMakeBorder( guide, temp2, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		splitBGRLineInterleave(temp2,tempg);
	}
	else if(cn==3 && cng==3)
	{
		Mat temp2;
		copyMakeBorder( src, temp2, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		splitBGRLineInterleave(temp2,temp);

		copyMakeBorder( guide, temp2, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		splitBGRLineInterleave(temp2,tempg);
	}
	else if(cn==3 && cng==1)
	{
		Mat temp2;
		copyMakeBorder( src, temp2, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		splitBGRLineInterleave(temp2,temp);

		copyMakeBorder( guide, tempg, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
	}

	double minv,maxv;
	minMaxLoc(guide,&minv,&maxv);
	const int color_range = cvRound(maxv-minv);

	vector<float> _color_weight(cng*color_range);
	vector<float> _space_weight(kernelSize.area()+1);
	vector<int> _space_ofs(kernelSize.area()+1);
	vector<int> _space_guide_ofs(kernelSize.area()+1);
	float* color_weight = &_color_weight[0];
	float* space_weight = &_space_weight[0];
	int* space_ofs = &_space_ofs[0];
	int* space_guide_ofs = &_space_guide_ofs[0];

	// initialize color-related bilateral filter coefficients

	for( i = 0; i < color_range*cng; i++ )
		color_weight[i] = (float)std::exp(i*i*gauss_color_coeff);

	// initialize space-related bilateral filter coefficients
	for( i = -radiusV, maxk = 0; i <= radiusV; i++ )
	{
		j = -radiusH;

		for( ;j <= radiusH; j++ )
		{
			double r = std::sqrt((double)i*i + (double)j*j);
			if( r > max(radiusV,radiusH) )
				continue;
			space_weight[maxk] = (float)std::exp(r*r*gauss_space_coeff);
			space_ofs[maxk] = (int)(i*temp.cols*cn + j);
			space_guide_ofs[maxk++] = (int)(i*tempg.cols*cng + j);
		}
	}

	Mat dest = Mat::zeros(Size(src.cols+dpad, src.rows),dst.type());
	JointBilateralFilter_32f_InvokerSSE4 body(dest, temp, tempg,radiusH,radiusV, maxk, space_ofs, space_guide_ofs,space_weight, color_weight);
	parallel_for_(Range(0, size.height), body);
	Mat(dest(Rect(0,0,dst.cols,dst.rows))).copyTo(dst);
}

void jointBilateralFilter_8u( const Mat& src, const Mat& guide, Mat& dst, Size kernelSize , double sigma_color, double sigma_space, int borderType )
{
	if(kernelSize.width==0 || kernelSize.height==0){ src.copyTo(dst);return;}
	int cn = src.channels();
	int cng = guide.channels();
	int i, j, maxk;
	Size size = src.size();

	CV_Assert( (src.type() == CV_8UC1   || src.type() == CV_8UC3) &&
		(guide.type() == CV_8UC1 || guide.type() == CV_8UC3) &&
		src.type() == dst.type() && src.size() == dst.size());

	if( sigma_color <= 0 )
		sigma_color = 1;
	if( sigma_space <= 0 )
		sigma_space = 1;

	double gauss_color_coeff = -0.5/(sigma_color*sigma_color);
	double gauss_space_coeff = -0.5/(sigma_space*sigma_space);

	int radiusH = kernelSize.width>>1;
	int radiusV = kernelSize.height>>1;

	Mat temp,tempg;

	int dpad = (16- src.cols%16)%16;
	int spad =  dpad + (16-(2*radiusH)%16)%16;
	if(spad<16) spad +=16;
	int lpad = 16*(radiusH/16+1)-radiusH;
	int rpad = spad-lpad;
	if(cn==1 && cng==1)
	{
		copyMakeBorder( src, temp, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		copyMakeBorder( guide, tempg, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
	}
	else if(cn==1 && cng==3)
	{
		copyMakeBorder( src, temp, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		Mat temp2;
		copyMakeBorder( guide, temp2, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		splitBGRLineInterleave(temp2,tempg);
	}
	else if(cn==3 && cng==3)
	{
		Mat temp2;
		copyMakeBorder( src, temp2, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		splitBGRLineInterleave(temp2,temp);

		copyMakeBorder( guide, temp2, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		splitBGRLineInterleave(temp2,tempg);
	}
	else if(cn==3 && cng==1)
	{
		Mat temp2;
		copyMakeBorder( src, temp2, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
		splitBGRLineInterleave(temp2,temp);

		copyMakeBorder( guide, tempg, radiusV, radiusV, radiusH+lpad, radiusH+rpad, borderType );
	}

	/*double minv,maxv;
	minMaxLoc(guide,&minv,&maxv);
	const int color_range = cvRound(maxv-minv);*/
	const int color_range = 256;

	vector<float> _color_weight(cng*color_range);
	vector<float> _space_weight(kernelSize.area()+1);
	vector<int> _space_ofs(kernelSize.area()+1);
	vector<int> _space_guide_ofs(kernelSize.area()+1);
	float* color_weight = &_color_weight[0];
	float* space_weight = &_space_weight[0];
	int* space_ofs = &_space_ofs[0];
	int* space_guide_ofs = &_space_guide_ofs[0];

	// initialize color-related bilateral filter coefficients

	for( i = 0; i < color_range*cng; i++ )
		color_weight[i] = (float)std::exp(i*i*gauss_color_coeff);

	// initialize space-related bilateral filter coefficients
	for( i = -radiusV, maxk = 0; i <= radiusV; i++ )
	{
		j = -radiusH;

		for( ;j <= radiusH; j++ )
		{
			double r = std::sqrt((double)i*i + (double)j*j);
			if( r > max(radiusV,radiusH) )
				continue;
			space_weight[maxk] = (float)std::exp(r*r*gauss_space_coeff);
			//space_ofs[maxk++] = (int)(i*temp.step + j*cn);
			space_ofs[maxk] = (int)(i*temp.cols*cn + j);
			space_guide_ofs[maxk++] = (int)(i*tempg.cols*cng + j);
		}
	}

	Mat dest = Mat::zeros(Size(src.cols+dpad, src.rows),dst.type());
	JointBilateralFilter_8u_InvokerSSE4 body(dest, temp, tempg,radiusH,radiusV, maxk, space_ofs, space_guide_ofs,space_weight, color_weight);
	parallel_for_(Range(0, size.height), body);
	Mat(dest(Rect(0,0,dst.cols,dst.rows))).copyTo(dst);
}

void jointBilateralFilterSP_32f( const Mat& src, const Mat& guide, Mat& dst, Size kernelSize, double sigma_color, double sigma_space, int borderType )
{
	jointBilateralFilter_32f(src, guide, dst, Size(kernelSize.width,1), sigma_color, sigma_space, borderType );
	jointBilateralFilter_32f(dst, guide, dst, Size(1,kernelSize.height), sigma_color, sigma_space, borderType );
}
void jointBilateralFilterSP_8u( const Mat& src, const Mat& guide, Mat& dst, Size kernelSize, double sigma_color, double sigma_space, int borderType )
{
	jointBilateralFilter_8u(src, guide, dst, Size(kernelSize.width,1), sigma_color, sigma_space, borderType );
	jointBilateralFilter_8u(dst, guide, dst, Size(1,kernelSize.height), sigma_color, sigma_space, borderType );
}


void jointBilateralFilter(const Mat& src, const Mat& guide, Mat& dst, Size kernelSize, double sigma_color, double sigma_space, int method, int borderType)
{
	if(dst.empty())dst.create(src.size(),src.type());
	if(method==BILATERAL_NORMAL)
	{
		if(src.type()==CV_MAKE_TYPE(CV_8U,src.channels()))
		{
			jointBilateralFilter_8u(src,guide,dst,kernelSize,sigma_color,sigma_space,borderType);
		}
		else if(src.type()==CV_MAKE_TYPE(CV_32F,src.channels()))
		{
			jointBilateralFilter_32f(src,guide,dst,kernelSize,sigma_color,sigma_space,borderType);
		}
		else
		{
			cout<<"use to 32f type to filter\n "<<endl;
			Mat srcf,guidef,dstf;
			
			src.convertTo(srcf,CV_32F);
			guide.convertTo(guidef,CV_32F);
			dstf = Mat::zeros(src.size(), srcf.type());
			jointBilateralFilter_32f(srcf,guidef,dstf,kernelSize,sigma_color,sigma_space,borderType);
			dstf.convertTo(dst,src.type());
		}
	}
	else if(method==BILATERAL_SEPARABLE)
	{
		if(src.type()==CV_MAKE_TYPE(CV_8U,src.channels()))
		{
			jointBilateralFilterSP_8u(src,guide,dst,kernelSize,sigma_color,sigma_space,borderType);
		}
		else if(src.type()==CV_MAKE_TYPE(CV_32F,src.channels()))
		{
			jointBilateralFilterSP_32f(src,guide,dst,kernelSize,sigma_color,sigma_space,borderType);
		}
	}
}

void weightedJointBilateralFilterSP_8u( const Mat& src, Mat& weightMap, const Mat& guide, Mat& dst, Size kernelSize, double sigma_color, double sigma_space, int borderType )
{
	weightedJointBilateralFilter_8u(src, weightMap, guide, dst, Size(kernelSize.width,1), sigma_color, sigma_space, borderType );
	weightedJointBilateralFilter_8u(dst, weightMap, guide, dst, Size(1,kernelSize.height), sigma_color, sigma_space, borderType );
}

void weightedJointBilateralFilterSP_32f( const Mat& src, Mat& weightMap, const Mat& guide, Mat& dst, Size kernelSize, double sigma_color, double sigma_space, int borderType )
{
	weightedJointBilateralFilter_32f(src, weightMap, guide, dst, Size(kernelSize.width,1), sigma_color, sigma_space, borderType );
	weightedJointBilateralFilter_32f(dst, weightMap, guide, dst, Size(1,kernelSize.height), sigma_color, sigma_space, borderType );
}
void weightedJointBilateralFilter(const Mat& src, Mat& weightMap,const Mat& guide, Mat& dst, Size kernelSize, double sigma_color, double sigma_space, int method, int borderType)
{
	if(dst.empty())dst.create(src.size(),src.type());
	if(method==BILATERAL_NORMAL)
	{
		if(src.type()==CV_MAKE_TYPE(CV_8U,src.channels()))
		{
			weightedJointBilateralFilter_8u(src,weightMap,guide,dst,kernelSize,sigma_color,sigma_space,borderType);
		}
		else if(src.type()==CV_MAKE_TYPE(CV_32F,src.channels()))
		{
			weightedJointBilateralFilter_32f(src,weightMap,guide,dst,kernelSize,sigma_color,sigma_space,borderType);
		}
	}
	else if(method==BILATERAL_SEPARABLE)
	{
		if(src.type()==CV_MAKE_TYPE(CV_8U,src.channels()))
		{
			weightedJointBilateralFilterSP_8u(src,weightMap,guide,dst,kernelSize,sigma_color,sigma_space,borderType);
		}
		else if(src.type()==CV_MAKE_TYPE(CV_32F,src.channels()))
		{
			weightedJointBilateralFilterSP_32f(src,weightMap,guide,dst,kernelSize,sigma_color,sigma_space,borderType);
		}
	}
}
