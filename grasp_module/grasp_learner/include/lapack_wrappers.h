#ifndef _lapack_wrappers_h_
#define _lapack_wrappers_h_

extern "C"
{


	static __inline void sgemm(const char *transa,const char *transb,int m,int n,int k,
				   float alpha,float *a,int lda,float *b,int ldb,
				   float beta,float *c,int ldc)
	{
		void sgemm_(const char *,const char *,int *,int *, int *,float *,float *,int *,
			    float *,int *,float *,float *,int *);
		
		sgemm_(transa,transb,&m,&n,&k,&alpha,a,&lda,b,&ldb,&beta,c,&ldc);
	}
	
	static __inline void ssyev(const char *jobz, const char *uplo, int n, float *a, int lda,
				   float *w, float *work, int lwork, int *info)
	{
		void ssyev_(const char*, const char*, int*, float*, int*,
			    float*, float*, int*, int*);
		ssyev_(jobz, uplo, &n, a, &lda, w, work, &lwork, info);
	}

}

#endif
