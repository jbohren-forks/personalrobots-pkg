/*
 * LevMarqSparseBundleAdj.cpp
 *
 *  Created on: Sep 24, 2008
 *      Author: jdchen
 */

#include "CvMat3X3.h"
#include "LevMarqSparseBundleAdj.h"
#include "PointTracks.h"
#include "boost/foreach.hpp"

#define DEBUG 1

namespace cv {
namespace willow {

LevMarqSparseBundleAdj::LevMarqSparseBundleAdj(const CvMat *disparityTo3D,
    const CvMat *threeDToDisparity,
    int full_free_window_size,
    int full_fixed_window_size,
    CvTermCriteria term_criteria):
      /// @todo move file term_criteria_ to Parent
      Parent(disparityTo3D, threeDToDisparity, term_criteria.max_iter, Rodrigues),
      full_free_window_size_(full_free_window_size),
      lowest_free_global_index_(-1),
      highest_free_global_index_(-1),
      full_fixed_window_size_(full_fixed_window_size),
      A_data_(new double[full_free_window_size*NUM_CAM_PARAMS*full_free_window_size*NUM_CAM_PARAMS]),
      A_step_(full_free_window_size*NUM_CAM_PARAMS),
      mat_A_full_(cvMat(full_free_window_size*NUM_CAM_PARAMS,
          full_free_window_size*NUM_CAM_PARAMS, CV_64FC1, A_data_)),
      B_data_(new double[full_free_window_size*NUM_CAM_PARAMS]),
      mat_B_full_(cvMat(full_free_window_size*NUM_CAM_PARAMS, 1, CV_64FC1, B_data_)),
      frame_params_(new double[full_free_window_size*NUM_CAM_PARAMS]),
      frame_prev_params_(new double[full_free_window_size*NUM_CAM_PARAMS]),
      frame_params_update_(new double[full_free_window_size*NUM_CAM_PARAMS]),
      free_transf_data_(new double[16*full_free_window_size]),
      fixed_transf_data_(new double[16*full_fixed_window_size]),
      transf_fwd_data_(new double[NUM_CAM_PARAMS*16*full_free_window_size]),
      lambdaLg10_(0),
      term_criteria_(term_criteria),
      param_delta_(CV_PI/(180.*100.)) /* this initial value still works even if we
      do not use angle explicitly */,
      num_retractions_(0),
      num_good_updates_(0)
      {
}

LevMarqSparseBundleAdj::~LevMarqSparseBundleAdj() {
  delete [] A_data_;
  delete [] B_data_;
  delete [] frame_params_;
  delete [] frame_prev_params_;
  delete [] frame_params_update_;
  delete [] free_transf_data_;
  delete [] fixed_transf_data_;
  delete [] transf_fwd_data_;
}

void LevMarqSparseBundleAdj::constructTransfMatrix(const CvMat* param, double _T[]){
  double _rt[16];
  CvMat rt;
  CvMat T;
  cvInitMatHeader(&rt, 4, 4, CV_64FC1, _rt);
  cvInitMatHeader(&T, 4, 4, CV_64FC1, _T);

  constructRTMatrix(param, _rt);
  _rt[15] = 1.0;
  _rt[12] = _rt[13] = _rt[14] = 0.0;

  cvMatMul(m3DToDisparity, &rt, &T);
#if DEBUG==1
  cout << "LevMarqSparseBundleAdj::constructTransfMatrix"<<endl;
  CvMatUtils::printMat(&T);
  cout << "="<<endl;
  CvMatUtils::printMat(m3DToDisparity);
  cout << "*"<<endl;
  CvMatUtils::printMat(&rt);
  #endif
}

void LevMarqSparseBundleAdj::constructFwdTransfMatrices(
    const CvMat *param, double delta,
    double *transf_fwds_data){
  double _param1[numParams];
  CvMat param1 = cvMat(numParams, 1, CV_64F, _param1);
  // transformation matrices for each parameter
  for (int k=0; k<numParams; k++) {
    // make a new copy for this parameter
    cvCopy(param, &param1);
    _param1[k] += delta;
    constructTransfMatrix(&param1, &transf_fwds_data[k*16]);
  }
}

void LevMarqSparseBundleAdj::constructTransfMatrices(){
  for (int iFrame=0; iFrame < free_window_size_; iFrame++) {
#if DEBUG==1
    printf("construct matrices for frame of local index: %d\n", iFrame);
#endif
    double *params_i = getFrameParams(iFrame);
    double *transf_data_i     = getFreeTransf(iFrame);
    CvMat mat_params_i = cvMat(NUM_CAM_PARAMS, 1, CV_64FC1, params_i);
    constructTransfMatrix((const CvMat *)&mat_params_i, transf_data_i);
  }
}


void LevMarqSparseBundleAdj::constructFwdTransfMatrices(){
  for (int iFrame=0; iFrame < free_window_size_; iFrame++) {
#if DEBUG==1
    printf("construct matrices for frame of local index: %d\n", iFrame);
#endif
    double *params_i = getFrameParams(iFrame);
    double *transf_fwd_data_i = getTransfFwd(iFrame, 0);
    CvMat mat_params_i = cvMat(NUM_CAM_PARAMS, 1, CV_64FC1, params_i);
    constructFwdTransfMatrices((const CvMat *)&mat_params_i,
        param_delta_, transf_fwd_data_i);
  }
}


bool LevMarqSparseBundleAdj::optimize(
    vector<FramePose*>* free_frames,
    vector<FramePose*>* fixed_frames,
    PointTracks* tracks
) {
  /// @todo add a member field is TrackPointObserv to indicate if
  /// the frame is free cam, fixed cam or don't care cam.
  cout << __PRETTY_FUNCTION__ << endl;

  if (free_frames->size()<1 || fixed_frames->size()<1) {
    return false;
  }

  /// set up matrices, and others
  free_window_size_ = free_frames->size();
  fixed_window_size_ = fixed_frames->size();
  num_tracks_  = tracks->tracks_.size();

  /// @todo get a more intelligent number than this arbitrary one.
  if (num_tracks_<10) {
    return false;
  }

#if DEBUG==1
  printf("number of tracks: %d\n", num_tracks_);
  printf("free window size: %d\n", free_window_size_);
#endif

  lowest_free_global_index_  = free_frames->front()->mIndex;
  highest_free_global_index_ = free_frames->back()->mIndex;
  cvGetSubRect(&mat_A_full_, &mat_A_, cvRect(0, 0,
      free_window_size_*NUM_CAM_PARAMS, free_window_size_*NUM_CAM_PARAMS));
  // for CvMat's that are actually one columns vectors, we do not need to use
  // cvGetSubRect()
  cvGetSubRect(&mat_B_full_, &mat_B_, cvRect(0, 0, 1, free_window_size_*NUM_CAM_PARAMS));
  mat_dC_      = cvMat(free_window_size_*NUM_CAM_PARAMS, 1, CV_64FC1, frame_params_update_);
  mat_C_       = cvMat(free_window_size_*NUM_CAM_PARAMS, 1, CV_64FC1, frame_params_);
  mat_prev_C_  = cvMat(free_window_size_*NUM_CAM_PARAMS, 1, CV_64FC1, frame_prev_params_);

  double Hcc2[NUM_CAM_PARAMS*NUM_CAM_PARAMS];
  CvMat  mat_Hcc2 = cvMat(NUM_CAM_PARAMS, NUM_CAM_PARAMS, CV_64FC1, Hcc2);

  double Hpp[NUM_POINT_PARAMS*NUM_POINT_PARAMS]; // the part of JtJ w.r.t. track p (or point p)
  double Hpp_inv[NUM_POINT_PARAMS*NUM_POINT_PARAMS];
  CvMat mat_Hpp     = cvMat(NUM_POINT_PARAMS, NUM_POINT_PARAMS, CV_64FC1, Hpp);
  CvMat mat_Hpp_inv;
  cvInitMatHeader(&mat_Hpp_inv, NUM_POINT_PARAMS, NUM_POINT_PARAMS, CV_64FC1, Hpp_inv);
  double bp[NUM_POINT_PARAMS];      // the part of bP  w.r.t. track p

#if DEBUG==1
  printf("address of Hpp_inv= 0x%x\n", (unsigned int) Hpp_inv);
#endif

  /// 1. Initialization of  \f$ \lambda \f$.
  lambdaLg10_ = -3;
  const double LOG10= log(10.);
  double lambda_plus_one = exp(lambdaLg10_*LOG10) + 1.0;
  double scale = 1./param_delta_;

  if( term_criteria_.type & CV_TERMCRIT_ITER )
    term_criteria_.max_iter = MIN(MAX(term_criteria_.max_iter,1),1000);
  else
    term_criteria_.max_iter = 30;
  if( term_criteria_.type & CV_TERMCRIT_EPS )
    term_criteria_.epsilon = MAX(term_criteria_.epsilon, 0);
  else
    term_criteria_.epsilon = DBL_EPSILON;

  // some statistics
  num_retractions_ = 0;
  num_good_updates_= 0;

  /// 2. Compute cost function at initial camera and point configuration.
  initCameraParams(free_frames, fixed_frames, tracks);

  /// For each camera/frame, compute the transformation matrix
  /// from global to disparity
  constructTransfMatrices();

  double cost = costFunction(free_frames, tracks);
  double prev_cost = DBL_MAX;

#if DEBUG==1
  printf("Initial cost: %f\n", cost);
#endif

  // 3x6 in stereo case
  CvMat* mat_Jc = cvCreateMat(DIM, NUM_CAM_PARAMS, CV_64FC1);
  double* Jc  = mat_Jc->data.db;
  // 3x3 in stereo case
  CvMat* mat_Jp = cvCreateMat(DIM, NUM_POINT_PARAMS, CV_64FC1);
  double* Jp = mat_Jp->data.db;

  // a fall back iteration bound in case of error in iteration control.
  int max_iter_for_safety  = term_criteria_.max_iter + defMaxTimesOfUpdates;

  /// Main loop of optimization.
  for (int iUpdates = 0, iters=0; iUpdates< max_iter_for_safety; iUpdates++ ) {
    /// 3. Clear the left hand side matrix A and right hand side vector B
    cvSetZero(&mat_A_);
    cvSetZero(&mat_B_);

    /// for computing numerical Jacobian,
    /// for each camera/frame compute the transformation matrices
    /// from global to disparity w.r.t. delta update on each camera parameter.
    constructFwdTransfMatrices();

    /// 4. For each track p {
    BOOST_FOREACH( PointTrack* p, tracks->tracks_) {
      /// - Compute the part of JtJ w.r.t to p.
      /// Clear a variable \f$ H_{pp} \f$ to represent block \f$ p \f$
      /// of \f$ H_{PP} \f$ (in our case a 3x3 matrix) and a variable
      /// \f$ b_p \f$ to represent part \f$ p \f$ of \f$ b_P \f$ (in our case
      /// a 3-vector)
      memset(Hpp, 0, NUM_POINT_PARAMS*NUM_POINT_PARAMS*sizeof(double));
      memset(bp,  0, NUM_POINT_PARAMS*sizeof(double));

      double px = p->coordinates_.x;
      double py = p->coordinates_.y;
      double pz = p->coordinates_.z;
#if DEBUG==1
      CvMat cart_point = cvMat(1, 1, CV_64FC1, &(p->coordinates_));
      printf("point %d: [%f, %f, %f]\n", p->id_, px, py, pz);
#endif

      /// - Compute derivatives.  For each camera c on track p.
      ///   {
      BOOST_FOREACH( PointTrackObserv* obsv, *p) {
        ///     Compute error vector f of reprojection in camera c of point p
        ///     and its Jacobian \f$ J_p \f$ and \f$ J_c\f$ with respect to the point parameters
        ///     (in our case 3x3 matrix) and the camera parameters (in out case
        ///     3x6 matrix). respectively.

        if (isDontCareFrame(obsv->frame_index_) == true) {
          continue;
        }

        double rx, ry, rz;
        double pu = obsv->disp_coord_.x;
        double pv = obsv->disp_coord_.y;
        double pd = obsv->disp_coord_.z;

#if 1 // the following shall be done by now
        // get a reference of the transformation matrix from global to disp
        int local_index = map_index_global_to_local_[obsv->frame_index_];
        double* transf_global_disp = getTransf(obsv->frame_index_, local_index);

        // compute the error vector for this point
        PERSTRANSFORMRESIDUE(transf_global_disp, px, py, pz, pu, pv, pd,
            rx, ry, rz);

        // @todo compute the err norm here, instead of doing it separately
#endif
#if DEBUG==1
        printf("obsv on frame %d: [%f, %f, %f] <=> err [%f, %f, %f]\n",
            obsv->frame_index_, pu, pv, pd, rx, ry, rz);
#endif
        // compute the residue w.r.t. the point parameters with a delta increment
        // in each parameter.
        // use the same buffer for Jacobian (3x3) of the error vector for point p.
        // The two dimensions are dim x param
        // for parameter x, fill out column 0 of Jp
        PERSTRANSFORMRESIDUE(transf_global_disp, px+param_delta_, py, pz, pu, pv, pd,
            Jp[0], Jp[3], Jp[6]);
        Jp[0] = (Jp[0] - rx)*scale;
        Jp[3] = (Jp[3] - ry)*scale;
        Jp[6] = (Jp[6] - rz)*scale;

        // for paramter y, fill out column 1 of Jp
        PERSTRANSFORMRESIDUE(transf_global_disp, px, py+param_delta_, pz, pu, pv, pd,
            Jp[1], Jp[4], Jp[7]);
        Jp[1] = (Jp[1] - rx)*scale;
        Jp[4] = (Jp[4] - ry)*scale;
        Jp[7] = (Jp[7] - rz)*scale;
        // for parameter z, fill out column 2 of Jp
        PERSTRANSFORMRESIDUE(transf_global_disp, px, py, pz+param_delta_, pu, pv, pd,
            Jp[2], Jp[5], Jp[8]);
        Jp[2] = (Jp[2] - rx)*scale;
        Jp[5] = (Jp[5] - ry)*scale;
        Jp[8] = (Jp[8] - rz)*scale;

#if DEBUG==1
        printf("Jacobian Jp of point track %d on frame %d, %d:\n", p->id_,
            obsv->frame_index_, obsv->local_frame_index_);
        CvMatUtils::printMat(mat_Jp);
#endif

        ///
        ///     Add \f$ J_p^T J_p\f$ to the upper triangular part of \f$ H_{pp} \f$
        for (int d0=0; d0<NUM_POINT_PARAMS; d0++) {
          double Jpx = Jp[                     d0];
          double Jpy = Jp[  NUM_POINT_PARAMS + d0];
          double Jpz = Jp[2*NUM_POINT_PARAMS + d0];
          for (int d1=d0; d1<NUM_POINT_PARAMS; d1++) {
            // Jp[d0]^T*Jp[d1], each a vector of 3
            Hpp[d0*NUM_POINT_PARAMS + d1] +=
              Jpx*Jp[                     d1] +
              Jpy*Jp[  NUM_POINT_PARAMS + d1] +
              Jpz*Jp[2*NUM_POINT_PARAMS + d1];
          }
          ///
          ///     Subtract \f$ J_p^T \f$ from \f$ b_p \f$.
          ///
          bp[d0] -= Jpx * rx + Jpy * ry + Jpz * rz;
        }

        ///     If camera c is free
        if (isFreeFrame(obsv->frame_index_) == true){
          /// Add \f$ J_c^TJ_c \f$ (optionally with an augmented diagonal)
          /// to upper triangular part of block (c, c) of
          /// left hand side matrix A (in our case 6x6 matrix).
          /// Compute block (p,c) of \f$ H_{PC} \f$ as H_{pc} = J_p^T J_c
          /// (in our case a 3x6 matrix) and store it until track is done.
          /// Subtract \f$ J_c^T f \f$ from part c of right hand side vector B
          /// (related to \f$ b_C \f$).

          // compute the residue w.r.t. the transformations with a delta increment
          // in each parameter.
          int frame_li = obsv->local_frame_index_;

          for (int k=0; k<NUM_CAM_PARAMS; k++) {
            double* transf_fwd_global_disp = getTransfFwd(frame_li, k);

            // fill out column k of Jc
            PERSTRANSFORMRESIDUE(transf_fwd_global_disp, px, py, pz, pu, pv, pd,
                Jc[k], Jc[NUM_CAM_PARAMS + k], Jc[2*NUM_CAM_PARAMS + k]);

            // compute the Jacobian regarding this point and this cam
            Jc[                   k] = (Jc[k]-rx)*scale;
            Jc[  NUM_CAM_PARAMS + k] = (Jc[  NUM_CAM_PARAMS + k]-ry)*scale;
            Jc[2*NUM_CAM_PARAMS + k] = (Jc[2*NUM_CAM_PARAMS + k]-rz)*scale;
          }
#if DEBUG==1
          {
            printf("Jacobian Jc of point %d, on frame %d,%d, error=[%f,%f,%f]\n",
                p->id_, obsv->frame_index_, obsv->local_frame_index_, rx, ry, rz);
            CvMatUtils::printMat(mat_Jc);
          }
#endif

          // update the JtJ entry corresponding to it, block (c, c)
          double *A_data_cc = getABlock(frame_li, frame_li);
          double *mat_B_data_c  = getBBlock(frame_li);
          double *Hpc = obsv->Hpc_;
          for (int k=0; k<NUM_CAM_PARAMS; k++) {
            double Jcx = Jc[k];
            double Jcy = Jc[k+NUM_CAM_PARAMS];
            double Jcz = Jc[k+2*NUM_CAM_PARAMS];
            // JtJ entry, H_{cc}, aka block(c,c)
            // augment the diagonal entries
            A_data_cc[k*A_step_ + k] +=
              lambda_plus_one * ( Jcx*Jcx + Jcy*Jcy + Jcz*Jcz);
            // off diagonal entries
            for (int l=k+1; l<NUM_CAM_PARAMS; l++) {
              A_data_cc[k*A_step_ + l] +=
                Jcx * Jc[l] + Jcy * Jc[l+NUM_CAM_PARAMS] + Jcz * Jc[l+2*NUM_CAM_PARAMS];
            }
            // H_{pc}, aka, block (p, c),
            for (int d=0; d<NUM_POINT_PARAMS; d++) {
              // compute entry (d, k) of Hpc = (col d of Jp)^T (col k of Jc)
              Hpc[d*NUM_CAM_PARAMS + k] =
                Jp[d]*Jcx + Jp[NUM_POINT_PARAMS + d]*Jcy + Jp[2*NUM_POINT_PARAMS + d]*Jcz;
            }
            // Subtract Jc^T f from part c of right hand side vector B
            mat_B_data_c[k] -= Jcx*rx + Jcy*ry + Jcz*rz;
#if DEBUG==1
            printf("row %d of B=%f, %f\n", k, mat_B_data_c[k], -(Jcx*rx + Jcy*ry + Jcz*rz));
#endif
          }
#if DEBUG==1
          {
            CvMat mat_Hpc = cvMat(3, 6, CV_64FC1, Hpc);
            printf("Hpc p=%d, c=%d,%d\n", p->id_, obsv->frame_index_, obsv->local_frame_index_);
            CvMatUtils::printMat(&mat_Hpc);
          }
#endif
        } // if camera c is free
      } // loop thru all observations of the track.

#if DEBUG==1
      printf("[LevMarqSBA] mat_A after Hcc updates for point %d\n", p->id_);
      CvMatUtils::printMat(&mat_A_);
      printf("[LevMarqSBA] mat_B after Hcc updates for point %d\n", p->id_);
      CvMatUtils::printMat(&mat_B_);
#endif


      /// Augment diagonal of \f$ H_{pp} \f$, which is now accumulated and ready.
      /// Invert \f$ H_{pp} \f$, taking advantage of the fact that is a symmetric
      /// matrix.  Note that \f$ H_{pp} \f$ is not needed anymore from this point on.
      /// We can reuse the space.

#if DEBUG==1
      printf("Hpp:\n");
      CvMatUtils::printMat(&mat_Hpp);
#endif

      // Augment diagonal of Hpp.
      for (int i=0; i<NUM_POINT_PARAMS; i++) {
        Hpp[i*NUM_POINT_PARAMS+i] *= lambda_plus_one;
      }
      // invert Hpp in place
      cvCompleteSymm(&mat_Hpp, 0);

      /// @todo Can cvInvert be done in place?
      cvInvert(&mat_Hpp, &mat_Hpp_inv, CV_SVD_SYM);
#if DEBUG==1
      printf("Hpp_inv augmented of p=%d, step=%d\n", p->id_, mat_Hpp_inv.step);
      CvMatUtils::printMat(&mat_Hpp_inv);
#endif

      /// Compute \f$ H_{pp}^{-1} b_p \f$ and store it in a variable \f$ t_p \f$.
      double* tp = p->tp_;
      for (int i=0; i<NUM_POINT_PARAMS; i++) {
        tp[i] = Hpp_inv[i*NUM_POINT_PARAMS +0] * bp[0] +
          Hpp_inv[i*NUM_POINT_PARAMS +1] * bp[1] + Hpp_inv[i*NUM_POINT_PARAMS +2]*bp[2];
      }

#if DEBUG==1
      printf("[LevMarqSBA] mat_B before Outer Product of Tracks\n");
      CvMatUtils::printMat(&mat_B_);
      printf("bp=[%f, %f, %f]\n", bp[0], bp[1], bp[2]);
      printf("tp=[%f, %f, %f]\n", tp[0], tp[1], tp[2]);
#endif
      /// (Outer product of track) For each free camera c on track p
      /// {
      for (PointTrack::iterator iObsv=p->begin(); iObsv!=p->end(); iObsv++) {
        PointTrackObserv* obsv = *iObsv;
        if (isFreeFrame(obsv->frame_index_) == false) {
          continue;
        }
        int local_index1 = obsv->local_frame_index_;
#if DEBUG==1
        printf("Outer product of track: fi=%d, lfi=%d\n", obsv->frame_index_, local_index1);
#endif
        double* Bc = getBBlock(local_index1);
        double* Hpc = obsv->Hpc_;
#if DEBUG==1
        {
          CvMat mat_Hpc = cvMat(3, 6, CV_64FC1, Hpc);
          printf("Hpc p=%d, c=%d,%d\n", p->id_, obsv->frame_index_, obsv->local_frame_index_);
          CvMatUtils::printMat(&mat_Hpc);
          printf("tp=[%f, %f, %f]\n", tp[0], tp[1], tp[2]);
        }
#endif
        ///   Subtract \f$ H_{pc}^T t_p = H_{pc}^T H_{pp}^{-1} b_p \f$ from part c
        ///   of right hand side vector B.
        CvMat&  mat_Hpc = obsv->mat_Hpc_;
        for (int i=0; i<NUM_CAM_PARAMS; i++) {
          Bc[i] -= Hpc[i]*tp[0] + Hpc[i + NUM_CAM_PARAMS]*tp[1] +
            Hpc[i + NUM_CAM_PARAMS*2]*tp[2];
#if DEBUG==1
            printf("row %d of B=%f, %f\n", i, Bc[i], -(Hpc[i]*tp[0] + Hpc[i + NUM_CAM_PARAMS]*tp[1] +
                Hpc[i + NUM_CAM_PARAMS*2]*tp[2]));
#endif

        }

        ///   Compute the matrix \f$ H_{pc}^T H_{pp}^{-1} and store it in a variable
        ///   \f$ T_{cp} \f$ (6x3).
        cvGEMM(&mat_Hpc, &mat_Hpp_inv, 1.0, NULL, 0.0, &obsv->mat_Tcp_, CV_GEMM_A_T);

#if DEBUG==1
        printf("matrix Hpc, p=%d, c=%d,%d\n", p->id_, obsv->frame_index_, local_index1);
        CvMatUtils::printMat(&mat_Hpc);
        printf("matrix Hpp_inv of p=%d\n", p->id_);
        CvMatUtils::printMat(&mat_Hpp_inv);
        printf("matrix Tcp\n");
        CvMatUtils::printMat(&obsv->mat_Tcp_);
#endif

#if DEBUG==1
        printf("[LevMarqSBA] mat_B before Hcc2\n");
        CvMatUtils::printMat(&mat_B_);
#endif
        ///   For each free camera c2 >= c on track p
        ///   {
        for (PointTrack::iterator iObsv2 = iObsv; iObsv2 != p->end(); iObsv2++) {
          PointTrackObserv* obsv2 = *iObsv2;
          if (isFreeFrame(obsv2->frame_index_)  == false) {
            continue;
          }
          int local_index2 = obsv2->local_frame_index_;
#if DEBUG==1
          printf("  fi=[%d,%d], lfi=[%d,%d]\n", obsv->frame_index_, obsv2->frame_index_,
              local_index1, local_index2);
#endif

          ///    Subtrack \f$ T_{pc}H_{pc2} = H__{pc}^T H_{pp}^{-1} H_{pc2} from
          ///     block (c, c2) of left hand side matrix A.
          CvMat& mat_Hpc2 = obsv2->mat_Hpc_;
          CvMat A_cc2;
          getABlock(&A_cc2, local_index1, local_index2);
          cvMatMul(&obsv->mat_Tcp_, &mat_Hpc2, &mat_Hcc2);
          cvSub(&A_cc2, &mat_Hcc2, &A_cc2);
#if DEBUG==1
          printf("matrix Tcp, p=%d, c=%d,%d\n", p->id_, obsv->frame_index_, local_index2);
          CvMatUtils::printMat(&obsv->mat_Tcp_);
          printf("matrix Hpc2, p=%d, c2=%d,%d\n", p->id_, obsv2->frame_index_, local_index2);
          CvMatUtils::printMat(&mat_Hpc2);
          printf("matrix Hcc2, c=%d,%d, c2=%d,%d\n", obsv->frame_index_, local_index1,
              obsv2->frame_index_, local_index2);
          CvMatUtils::printMat(&mat_Hcc2);
#endif
        }
        ///   }
      } // (Outer product of track)
      /// }
#if DEBUG==1
      printf("[LevMarqSBA] mat_A after Outer Product of Tracks\n");
      CvMatUtils::printMat(&mat_A_);
      printf("[LevMarqSBA] mat_B after Outer Product of Tracks\n");
      CvMatUtils::printMat(&mat_B_);
#endif

    } // done with a point track.
    /// 5. (Optional) Fix gauge by freezing appropriate coordinates and
    /// therefore reducing the linear system with a few dimensions.
    ///
    /// 6. (Linear Solving) Cholesky factor the left hand side matrix A and
    /// solve for dC. Add the frozen coordinates back in.
    /// (Instead we use SVD for symmetric square matrix
    // fill out of lower left part of the matrix
    cvCompleteSymm(&mat_A_, 0);
    cvSolve(&mat_A_, &mat_B_, &mat_dC_, CV_SVD_SYM);
#if DEBUG==1
    printf("[LevMarqSBA]: cvSolve\n");
    printf("[LevMarqSBA]: matrix A:\n");
    CvMatUtils::printMat(&mat_A_);
    printf("[LevMarqSBA]: matrix B:\n");
    CvMatUtils::printMat(&mat_B_);
    printf("[LevMarqSBA]: matrix dC:\n");
    CvMatUtils::printMat(&mat_dC_);
#endif


    /// update camera parameters with mat_dC
    cvAdd(&mat_C_, &mat_dC_, &mat_C_);
#if DEBUG==1
    printf("[LevMarqSBA]: updated cam params\n");
    CvMatUtils::printMat(&mat_C_);
#endif

    ///
    /// 7. (Backsubstitution)  for each track p
    /// {
    BOOST_FOREACH( PointTrack* p, tracks->tracks_) {
      ///   Start with point update for this track dp = tp
      double* dp = p->dp_;
      CvMat& mat_dp = p->mat_dp_;
      for (int i = 0; i<NUM_POINT_PARAMS; i++) {
        dp[i] = p->tp_[i];
      }
#if DEBUG==1
      printf("[LevMarqSBA] backsubstitution, point %d, initial dp\n", p->id_);
      CvMatUtils::printMat(&mat_dp);
#endif
      /// for  each free camera c on track p
      /// {
      BOOST_FOREACH( PointTrackObserv* obsv, *p ) {
        if (isFreeFrame(obsv->frame_index_)  == false) {
          continue;
        }
        ///    Subtract T_{cp}^T dc from dp (where dc is the update for camera c).
        double* dc = getFrameParamsUpdate(obsv->local_frame_index_);
        CvMat mat_dc  = cvMat(NUM_CAM_PARAMS, 1, CV_64FC1, dc);
        CvMat& mat_Tcp = obsv->mat_Tcp_;
        cvGEMM(&mat_Tcp, &mat_dc, -1.0, &mat_dp, 1.0, &mat_dp, CV_GEMM_A_T);

#if DEBUG==1
        printf("[LevMarqSBA] backsubstitution, point %d, cam %d,%d\n", p->id_,
            obsv->frame_index_, obsv->local_frame_index_);
        printf("[LevMarqSBA] Tcp\n");
        CvMatUtils::printMat(&mat_Tcp);
        printf("[LevMarqSBA] dc\n");
        CvMatUtils::printMat(&mat_dc);
        printf("[LevMarqSBA] updated dp\n");
        CvMatUtils::printMat(&mat_dp);

#endif
      }

      /// update point parameters with dp
      p->coordinates_.x += dp[0];
      p->coordinates_.y += dp[1];
      p->coordinates_.z += dp[2];

#if DEBUG==1
      printf("[LevMarqSBA]: updated point params pid=%d, [%f, %f, %f]\n",
          p->id_, p->coordinates_.x, p->coordinates_.y, p->coordinates_.z);
#endif

      /// }
    }
    ///
    /// 8. Compute the cost function for the updated camera and point configuration
    constructTransfMatrices();
    cost = costFunction(free_frames, tracks);
    printf("[LevMarqSBA] cost of iteration %d, %d = %e <=> %e (prev)\n", iUpdates, iters, cost, prev_cost);
    ///
    if (cost <= prev_cost) {
      /// 9. If cost function has improved, accept the update step, decrease
      ///    \f$ \lambda \f$ and go to Step 3 (unless converged, in which case quit).
      lambdaLg10_ = MAX(lambdaLg10_-1, -16);
      num_good_updates_++;

      /// check for convergence
      double param_change=0;
      if( ++iters >= term_criteria_.max_iter ||
          (param_change = getParamChange(tracks)) < term_criteria_.epsilon )
      {
        // Done!
#if DEBUG==1
        printf("[LevMarqSBA] Optimization Done. num of iters=%d >= %d || change in param=%e < %e\n",
            iters, term_criteria_.max_iter,param_change, term_criteria_.epsilon);
#endif
        break;
      }
#if DEBUG==1
      printf("[LevMarqSBA] good update. num of iters=%d, change in param=%e <=> %e\n",
          iters, param_change, term_criteria_.epsilon);
#endif

      prev_cost = cost;
      // accept parameter changes
      cvCopy(&mat_C_, &mat_prev_C_);
      BOOST_FOREACH(PointTrack* p, tracks->tracks_) {
        p->prev_coordinates_ = p->coordinates_;
      }
    } else {
      /// 10. Otherwise, reject the update,
      /// increase \f$ \lambda \f$ and go to Step 3 (unless exceeded the
      /// maximum number of iterations, in which case quit).
      lambdaLg10_++;
      num_retractions_++;

      /// back off from current parameters to previous ones?
      cvCopy(&mat_prev_C_, &mat_C_);
      BOOST_FOREACH(PointTrack* p, tracks->tracks_) {
        p->coordinates_ = p->prev_coordinates_;
      }
#if DEBUG==1
      printf("[LevMarqSBA] bad update.\n");
#endif

      /// @todo, in stead of just going back to the beginning of the loop,
      /// we shall skip over the computation of derivatives and Jacobians, etc..
      /// what we really need is just updating the diagonal elements.
      /// With this, we may be able to save parameter backing off in last step.

    }

    // update lambda according to lambdalog10
    lambda_plus_one = exp(lambdaLg10_*LOG10) + 1.0;
  } // next iteration

  /// copy the optimized parameters back
  retrieveOptimizedParams(free_frames, tracks);

#if DEBUG==1
  printf("[LevMarqSBA]:  Number of retractions=%d, number of good updates=%d\n",
      num_retractions_, num_good_updates_);
#endif
  cvReleaseMat(&mat_Jp);
  cvReleaseMat(&mat_Jc);
  return true;
}

void LevMarqSparseBundleAdj::initCameraParams(
    vector<FramePose*>* free_frames,
    vector<FramePose*>* fixed_frames,
    PointTracks* tracks) {
    int oldest_frame_in_tracks = tracks->oldest_frame_index_in_tracks_;
#if DEBUG==1
  cout << "initCameraParams(): oldest frame index in track: "<<
  oldest_frame_in_tracks<<endl;
#endif
  /// For each free frame (camera), set up transformation from global coordinates
  /// to disparity space. And transformation parameters.
  int local_index=0;
  double global_to_local_data[16];
  CvMat  global_to_local = cvMat(4, 4, CV_64FC1, global_to_local_data);
  BOOST_FOREACH(FramePose* free_frame, *free_frames) {
    // invert the global matrix to global to local matrix
    CvMatUtils::invertRigidTransform( &(free_frame->transf_local_to_global_),
        &global_to_local);

    // Convert from global_to_local transformation to optimization parameter of choice,
    // e.g. Euler angle or Rodrigues.
    // compute the rodrigues and shift vectors, which are used as
    // optimization parameters.
    double* frame_params_i = getFrameParams(local_index);
    transfToParams(global_to_local, frame_params_i);
#if   0 //@todo remove it
    CvMat mat_frame_params_i = cvMat(6, 1, CV_64FC1, getFrameParams(local_index));
    CvMatUtils::transformToRodriguesAndShift(
        global_to_local, mat_frame_params_i);
#endif

    // enter the mapping between global index and local index to the map.
    map_index_global_to_local_[free_frame->mIndex] = local_index;

    local_index++;
  }
  /// copy init parameters from mat_C_ to mat_prev_C_
  cvCopy(&mat_C_, &mat_prev_C_);

#if DEBUG==1
  {
    printf("Initial free camera parameters\n");
    CvMatUtils::printMat(&mat_C_);
  }
#endif

  // compute and enter the transformations of the fixed frames/cameras.
//  int lowest_index_in_window = free_frames->front()->mFrameIndex;
  // compute the size of the fixed camera window.
  fixed_window_size_ = fixed_frames->size();
  lowest_fixed_global_index_ = fixed_frames->front()->mIndex;
  highest_fixed_global_index_ = fixed_frames->back()->mIndex;

#if 0 // do not need the following
  BOOST_REVERSE_FOREACH(FramePose* fp, *fixed_frames) {
    if (fp->mIndex < oldest_frame_in_tracks) {
      break;
    } else if (fixed_window_size_ >= full_fixed_window_size_) {
      break;
    } else if (fp->mIndex < lowest_index_in_window) {
      fixed_window_size_++;
      lowest_fixed_global_index_ = fp->mIndex;
    }
  }
#endif
  CvMat* transf_from_global = cvCreateMat(4, 4, CV_64FC1);
  int reverse_index=0;
  BOOST_REVERSE_FOREACH(FramePose* fp, *fixed_frames) {
    // @todo we may not need this field transf_global_to_disp_
    if (fp->transf_global_to_disp_ == NULL ) {
      fp->transf_global_to_disp_ = cvCreateMat(4, 4, CV_64FC1);

      CvMatUtils::invertRigidTransform(&fp->transf_local_to_global_,
          transf_from_global);
      cvMatMul(m3DToDisparity, transf_from_global, fp->transf_global_to_disp_);
    }

    int local_index  = fixed_window_size_ - 1 - reverse_index;
    double* transf   = getFixedTransf(local_index);
    CvMat mat_transf = cvMat(4, 4, CV_64FC1, transf);
    // make a copy
    cvCopy(fp->transf_global_to_disp_,  &mat_transf );

#if DEBUG==1
    printf("[LevMarqSBA] Store global to disp transf of fixed frame: %d, local index=%d\n",
        fp->mIndex, local_index);
    CvMatUtils::printMat(&mat_transf);
#endif
    map_index_global_to_local_[fp->mIndex] = local_index;

    reverse_index++;
    assert(reverse_index<=fixed_window_size_);
  }

  // loop thru the tracks and fill out the field of local_frame_index_
  // according to map_index_global_to_local_
  BOOST_FOREACH( PointTrack* p, tracks->tracks_) {
    /// initialize prev_coordinates to the same as initial point params.
    p->prev_coordinates_ = p->coordinates_;
    BOOST_FOREACH( PointTrackObserv* obsv, *p ) {
      boost::unordered_map<int, int>::const_iterator iter =
        map_index_global_to_local_.find(obsv->frame_index_);
      if (iter != map_index_global_to_local_.end()) {
        obsv->local_frame_index_ = iter->second;
#if DEBUG==1
        printf("store local index %d for frame %d in track %d\n",
            obsv->local_frame_index_, obsv->frame_index_, p->id_);
#endif
      } else {
        obsv->local_frame_index_ = -1;
      }
    }
  }

  cvReleaseMat(&transf_from_global);
}

void LevMarqSparseBundleAdj::retrieveOptimizedParams(
    vector<FramePose *>* free_frames,
    PointTracks* tracks
){
  double transf_global_to_local_data[4*4];
  CvMat transf_global_to_local = cvMat(4, 4, CV_64FC1, transf_global_to_local_data);

  double params_local_to_global_data[NUM_CAM_PARAMS];
  CvMat  params_local_to_global = cvMat(NUM_CAM_PARAMS, 1, CV_64FC1, params_local_to_global_data);
  BOOST_FOREACH(FramePose* fp, *free_frames) {
    int local_index = map_index_global_to_local_[fp->mIndex];
    // copy the parameters out
    CvMat mat_params_i = cvMat(NUM_CAM_PARAMS, 1, CV_64FC1, getFrameParams(local_index));
    CvMatUtils::transformFromRodriguesAndShift(mat_params_i, transf_global_to_local);

    // compute the local to global matrix
    CvMatUtils::invertRigidTransform(&transf_global_to_local,
        &fp->transf_local_to_global_);

    // @todo, I do not think it is this function's job to do the following update.
    // update fp->mRod and fp->mShift
    CvMatUtils::transformToRodriguesAndShift(fp->transf_local_to_global_, params_local_to_global);
    fp->mRod.x = params_local_to_global_data[0];
    fp->mRod.y = params_local_to_global_data[1];
    fp->mRod.z = params_local_to_global_data[2];

    fp->mShift.x = params_local_to_global_data[3];
    fp->mShift.y = params_local_to_global_data[4];
    fp->mShift.z = params_local_to_global_data[5];
  }

#if 0 // remove
  // make a copy to frame poses as well.
  // keep the two for loop separate, as we may remove the references  of
  // PoseEstFrameEntry from this class
  double params_local_to_global_data[NUM_CAM_PARAMS];
  CvMat  params_local_to_global = cvMat(NUM_CAM_PARAMS, 1, CV_64FC1, params_local_to_global_data);
  BOOST_REVERSE_FOREACH(FramePose* fp, *frame_poses) {
    if (fp->mIndex > highest_free_global_index_ ) {
      continue;
    } else if (fp->mIndex < lowest_free_global_index_) {
      break;
    } else {
      int local_index = map_index_global_to_local_[fp->mIndex];
      // copy the parameters out
      CvMat mat_params_i = cvMat(NUM_CAM_PARAMS, 1, CV_64FC1, getFrameParams(local_index));
      CvMatUtils::transformFromRodriguesAndShift(mat_params_i, transf_global_to_local);

      // compute the local to global matrix
      CvMatUtils::invertRigidTransform(&transf_global_to_local,
          &fp->transf_local_to_global_);

      // update fp->mRod and fp->mShift
      CvMatUtils::transformToRodriguesAndShift(fp->transf_local_to_global_, params_local_to_global);
      fp->mRod.x = params_local_to_global_data[0];
      fp->mRod.y = params_local_to_global_data[1];
      fp->mRod.z = params_local_to_global_data[2];

      fp->mShift.x = params_local_to_global_data[3];
      fp->mShift.y = params_local_to_global_data[4];
      fp->mShift.z = params_local_to_global_data[5];
    }
  }
#endif

  /// @todo retrieve point parameters. Current implementation has
  /// the parameters on the tracks. So it is a noop.
#if 0
  // update update disp_coord_est_ of each obsv
  BOOST_FOREACH(PointTrack* p, tracks->tracks_) {
    printf("update point track %3d [%8.2f, %8.2f, %8.2f]\n",
        p->id_, p->coordinates_.x, p->coordinates_.y, p->coordinates_.z);
    BOOST_FOREACH(PointTrackObserv* obsv, *p) {

      if (isDontCareFrame(obsv->frame_index_) == true) {
        // we do not update this frame anymore.
        continue;
      }

      printf("fi=%3d [%8.2f, %8.2f, %8.2f] free=%d, local_index=%d\n",
          obsv->frame_index_, obsv->disp_coord_.x, obsv->disp_coord_.y,
          obsv->disp_coord_.z, isFreeFrame(obsv->frame_index_), obsv->local_frame_index_);


      double *transf_global_to_disp = getTransf(obsv->frame_index_, obsv->local_frame_index_);

      CvMat mat_global_to_disp = cvMat(4, 4, CV_64FC1, transf_global_to_disp);
      CvMat mat_coord = cvMat(1, 1, CV_64FC3, &p->coordinates_);
      CvMat mat_disp_coord_est_ = cvMat(1, 1, CV_64FC3, &obsv->disp_coord_est_);

      cvPerspectiveTransform(&mat_coord, &mat_disp_coord_est_, &mat_global_to_disp);
#if 1
      printf("global to disp for frame: %d, 0x%x\n", obsv->frame_index_,
          (unsigned int)transf_global_to_disp);
      CvMatUtils::printMat(&mat_global_to_disp);

      printf("estimated disp: [%8.2f, %8.2f, %8.2f]\n", obsv->disp_coord_est_.x,
          obsv->disp_coord_est_.y, obsv->disp_coord_est_.z);
#endif

    }
  }
#endif

}

double LevMarqSparseBundleAdj::costFunction(
    vector<FramePose *>* free_frames,
    PointTracks* tracks
) {
  double err_norm = 0;

  /// For each free frame (camera), set up transformation from global coordinates
  /// to disparity space.
  /// @todo the following is not necessary. Remove it and remove map_global_to_disp
#if 0
  BOOST_FOREACH(PoseEstFrameEntry* pose_est_frame_entry, *windowOfFrames) {
    CvMatUtils::transformFromRodriguesAndShift(
        pose_est_frame_entry->mTransformParams,
        pose_est_frame_entry->transf_global_to_local_);
    // lastly, global to local disparity
    cvMatMul(m3DToDisparity, &pose_est_frame_entry->transf_global_to_local_,
        &pose_est_frame_entry->mTransformGlobalToDisp);
    // enter the global to disp transformation into the hash map. again and again???
    map_global_to_disp_[pose_est_frame_entry->mFrameIndex] =
      &pose_est_frame_entry->mTransformGlobalToDisp;
  }
#endif

  /// For each tracks
  BOOST_FOREACH(PointTrack* track, tracks->tracks_) {
    CvMat point = cvMat(1, 1, CV_64FC3, &track->coordinates_);
    /// For each observation of the track
    BOOST_FOREACH(PointTrackObserv* obsv, *track) {
      if (isDontCareFrame(obsv->frame_index_) == true) {
        // we do not consider this frame anymore.
        continue;
      }

      CvMat disp_point = cvMat(1, 1, CV_64FC3, &obsv->disp_coord_);
      CvMat disp_point_est = cvMat(1, 1, CV_64FC3, &obsv->disp_coord_est_);

      double *transf_global_to_disp = getTransf(obsv->frame_index_, obsv->local_frame_index_);

      CvMat mat_global_to_disp = cvMat(4, 4, CV_64FC1, transf_global_to_disp);

      cvPerspectiveTransform(&point, &disp_point_est, &mat_global_to_disp);
      double dx = obsv->disp_coord_.x - obsv->disp_coord_est_.x;
      double dy = obsv->disp_coord_.y - obsv->disp_coord_est_.y;
      double dz = obsv->disp_coord_.z - obsv->disp_coord_est_.z;
      err_norm += dx*dx + dy*dy + dz*dz;
    }
  }

  return err_norm;
}

double LevMarqSparseBundleAdj::getParamChange(const PointTracks* tracks) const {
  // 2-norm of the camera parameters
  double frame_param_diff_sum_sq;
  double frame_param_norm_sq;
  double point_param_diff_sum_sq;
  double point_param_norm_sq;

  frame_param_diff_sum_sq  = cvNorm(&mat_C_, &mat_prev_C_, CV_L2);
  frame_param_diff_sum_sq *= frame_param_diff_sum_sq;
  frame_param_norm_sq         = cvNorm(&mat_C_, NULL, CV_L2);
  frame_param_norm_sq        *= frame_param_norm_sq;

  getPointParamChange(tracks, &point_param_diff_sum_sq, &point_param_norm_sq);

  double param_diff_norm = frame_param_diff_sum_sq + point_param_diff_sum_sq;
  param_diff_norm /= frame_param_norm_sq + point_param_norm_sq;
  param_diff_norm = sqrt(param_diff_norm);
#if DEBUG==1
  printf("parameter change (relative L2 norm of diff): %e\n", param_diff_norm);
#endif
  return param_diff_norm;
}

void LevMarqSparseBundleAdj::getPointParamChange(
    const PointTracks* tracks,
    double *param_diff_sum_sq,
    double *param_sum_sq) const {
  /// For each tracks
  double diff_sum_sq = 0.;
  double sum_sq = 0.;
  BOOST_FOREACH(const PointTrack* track, tracks->tracks_) {
    double x = track->coordinates_.x;
    double y = track->coordinates_.y;
    double z = track->coordinates_.z;
    sum_sq = x*x + y*y + z*z;

    // recycle the variables for storage of the diffs
    x -= track->prev_coordinates_.x;
    y -= track->prev_coordinates_.y;
    z -= track->prev_coordinates_.z;

    diff_sum_sq = x*x + y*y + z*z;

    *param_diff_sum_sq = diff_sum_sq;
    *param_sum_sq      = sum_sq;
  }
}

}

}
