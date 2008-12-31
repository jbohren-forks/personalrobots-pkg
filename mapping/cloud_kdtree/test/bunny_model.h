/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: bunny_model.h 8160 2008-12-16 07:41:55Z veedee $
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _CLOUD_KDTREE_BUNNYMODEL_H
#define _CLOUD_KDTREE_BUNNYMODEL_H_

#include "std_msgs/PointCloud.h"

namespace cloud_kdtree_tests
{

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Fill in a given point cloud message data structure with a downsampled copy of the Stanford bunny model
    * \param points the cloud data
    */
  void
    getBunnyModel (std_msgs::PointCloud &points)
  {
    points.pts.resize (397);

    points.pts[    0].x = 0.005422;  points.pts[    0].y = 0.113490; points.pts[    0].z = 0.040749;
    points.pts[    1].x = -0.00174;  points.pts[    1].y = 0.114250; points.pts[    1].z = 0.041273;
    points.pts[    2].x = -0.01066;  points.pts[    2].y = 0.113380; points.pts[    2].z = 0.040916;
    points.pts[    3].x = 0.026422;  points.pts[    3].y = 0.114990; points.pts[    3].z = 0.032623;
    points.pts[    4].x = 0.024545;  points.pts[    4].y = 0.122840; points.pts[    4].z = 0.024255;
    points.pts[    5].x = 0.034137;  points.pts[    5].y = 0.113160; points.pts[    5].z = 0.025070;
    points.pts[    6].x = 0.028860;  points.pts[    6].y = 0.117730; points.pts[    6].z = 0.027037;
    points.pts[    7].x = 0.026750;  points.pts[    7].y = 0.122340; points.pts[    7].z = 0.017605;
    points.pts[    8].x = 0.035750;  points.pts[    8].y = 0.112300; points.pts[    8].z = 0.019109;
    points.pts[    9].x = 0.015982;  points.pts[    9].y = 0.123070; points.pts[    9].z = 0.031279;
    points.pts[   10].x = 0.007981;  points.pts[   10].y = 0.124380; points.pts[   10].z = 0.032798;
    points.pts[   11].x = 0.018101;  points.pts[   11].y = 0.116740; points.pts[   11].z = 0.035493;
    points.pts[   12].x = 0.008669;  points.pts[   12].y = 0.117580; points.pts[   12].z = 0.037538;
    points.pts[   13].x = 0.018080;  points.pts[   13].y = 0.125360; points.pts[   13].z = 0.026132;
    points.pts[   14].x = 0.008086;  points.pts[   14].y = 0.128660; points.pts[   14].z = 0.026190;
    points.pts[   15].x = 0.022750;  points.pts[   15].y = 0.121460; points.pts[   15].z = 0.029671;
    points.pts[   16].x = -0.001869; points.pts[   16].y = 0.124560; points.pts[   16].z = 0.033184;
    points.pts[   17].x = -0.011168; points.pts[   17].y = 0.123760; points.pts[   17].z = 0.032519;
    points.pts[   18].x = -0.002006; points.pts[   18].y = 0.119370; points.pts[   18].z = 0.038104;
    points.pts[   19].x = -0.012320; points.pts[   19].y = 0.118160; points.pts[   19].z = 0.037427;
    points.pts[   20].x = -0.001666; points.pts[   20].y = 0.128790; points.pts[   20].z = 0.026782;
    points.pts[   21].x = -0.011971; points.pts[   21].y = 0.127230; points.pts[   21].z = 0.026219;
    points.pts[   22].x = 0.016484;  points.pts[   22].y = 0.128280; points.pts[   22].z = 0.019280;
    points.pts[   23].x = 0.007092;  points.pts[   23].y = 0.131030; points.pts[   23].z = 0.018415;
    points.pts[   24].x = 0.001461;  points.pts[   24].y = 0.131340; points.pts[   24].z = 0.017095;
    points.pts[   25].x = -0.013821; points.pts[   25].y = 0.128860; points.pts[   25].z = 0.019265;
    points.pts[   26].x = -0.017250; points.pts[   26].y = 0.112020; points.pts[   26].z = 0.040077;
    points.pts[   27].x = -0.074556; points.pts[   27].y = 0.134150; points.pts[   27].z = 0.051046;
    points.pts[   28].x = -0.065971; points.pts[   28].y = 0.143960; points.pts[   28].z = 0.041090;
    points.pts[   29].x = -0.071925; points.pts[   29].y = 0.145450; points.pts[   29].z = 0.043266;
    points.pts[   30].x = -0.065510; points.pts[   30].y = 0.136240; points.pts[   30].z = 0.042195;
    points.pts[   31].x = -0.071112; points.pts[   31].y = 0.137670; points.pts[   31].z = 0.047518;
    points.pts[   32].x = -0.079528; points.pts[   32].y = 0.134160; points.pts[   32].z = 0.051194;
    points.pts[   33].x = -0.080421; points.pts[   33].y = 0.144280; points.pts[   33].z = 0.042793;
    points.pts[   34].x = -0.082672; points.pts[   34].y = 0.137800; points.pts[   34].z = 0.046806;
    points.pts[   35].x = -0.088130; points.pts[   35].y = 0.135140; points.pts[   35].z = 0.042222;
    points.pts[   36].x = -0.066325; points.pts[   36].y = 0.123470; points.pts[   36].z = 0.050729;
    points.pts[   37].x = -0.072399; points.pts[   37].y = 0.126620; points.pts[   37].z = 0.052364;
    points.pts[   38].x = -0.066091; points.pts[   38].y = 0.119730; points.pts[   38].z = 0.050881;
    points.pts[   39].x = -0.072012; points.pts[   39].y = 0.118110; points.pts[   39].z = 0.052295;
    points.pts[   40].x = -0.062433; points.pts[   40].y = 0.126270; points.pts[   40].z = 0.043831;
    points.pts[   41].x = -0.068326; points.pts[   41].y = 0.129980; points.pts[   41].z = 0.048875;
    points.pts[   42].x = -0.063094; points.pts[   42].y = 0.118110; points.pts[   42].z = 0.044399;
    points.pts[   43].x = -0.071301; points.pts[   43].y = 0.113220; points.pts[   43].z = 0.048410;
    points.pts[   44].x = -0.080515; points.pts[   44].y = 0.127410; points.pts[   44].z = 0.052034;
    points.pts[   45].x = -0.078179; points.pts[   45].y = 0.119100; points.pts[   45].z = 0.051116;
    points.pts[   46].x = -0.085216; points.pts[   46].y = 0.126090; points.pts[   46].z = 0.049001;
    points.pts[   47].x = -0.089538; points.pts[   47].y = 0.126210; points.pts[   47].z = 0.044589;
    points.pts[   48].x = -0.082659; points.pts[   48].y = 0.116610; points.pts[   48].z = 0.047970;
    points.pts[   49].x = -0.089536; points.pts[   49].y = 0.117840; points.pts[   49].z = 0.044570;
    points.pts[   50].x = -0.056500; points.pts[   50].y = 0.152480; points.pts[   50].z = 0.030132;
    points.pts[   51].x = -0.055517; points.pts[   51].y = 0.153130; points.pts[   51].z = 0.026915;
    points.pts[   52].x = -0.036250; points.pts[   52].y = 0.171980; points.pts[   52].z = 0.000177;
    points.pts[   53].x = -0.037750; points.pts[   53].y = 0.171980; points.pts[   53].z = 0.000222;
    points.pts[   54].x = -0.036250; points.pts[   54].y = 0.169350; points.pts[   54].z = 0.000520;
    points.pts[   55].x = -0.033176; points.pts[   55].y = 0.157110; points.pts[   55].z = 0.001868;
    points.pts[   56].x = -0.051913; points.pts[   56].y = 0.154500; points.pts[   56].z = 0.011273;
    points.pts[   57].x = -0.041707; points.pts[   57].y = 0.166420; points.pts[   57].z = 0.003052;
    points.pts[   58].x = -0.049468; points.pts[   58].y = 0.164140; points.pts[   58].z = 0.004199;
    points.pts[   59].x = -0.041892; points.pts[   59].y = 0.156690; points.pts[   59].z = 0.005488;
    points.pts[   60].x = -0.051224; points.pts[   60].y = 0.158780; points.pts[   60].z = 0.008028;
    points.pts[   61].x = -0.062417; points.pts[   61].y = 0.153170; points.pts[   61].z = 0.033161;
    points.pts[   62].x = -0.071670; points.pts[   62].y = 0.153190; points.pts[   62].z = 0.033701;
    points.pts[   63].x = -0.062543; points.pts[   63].y = 0.155240; points.pts[   63].z = 0.027405;
    points.pts[   64].x = -0.072110; points.pts[   64].y = 0.155500; points.pts[   64].z = 0.027645;
    points.pts[   65].x = -0.078663; points.pts[   65].y = 0.152690; points.pts[   65].z = 0.032268;
    points.pts[   66].x = -0.081569; points.pts[   66].y = 0.153740; points.pts[   66].z = 0.026085;
    points.pts[   67].x = -0.087250; points.pts[   67].y = 0.152300; points.pts[   67].z = 0.022135;
    points.pts[   68].x = -0.057250; points.pts[   68].y = 0.155680; points.pts[   68].z = 0.010325;
    points.pts[   69].x = -0.057888; points.pts[   69].y = 0.157500; points.pts[   69].z = 0.007322;
    points.pts[   70].x = -0.088500; points.pts[   70].y = 0.152230; points.pts[   70].z = 0.019215;
    points.pts[   71].x = -0.056129; points.pts[   71].y = 0.146160; points.pts[   71].z = 0.030850;
    points.pts[   72].x = -0.054705; points.pts[   72].y = 0.135550; points.pts[   72].z = 0.032127;
    points.pts[   73].x = -0.054144; points.pts[   73].y = 0.147140; points.pts[   73].z = 0.026275;
    points.pts[   74].x = -0.046625; points.pts[   74].y = 0.132340; points.pts[   74].z = 0.021909;
    points.pts[   75].x = -0.051390; points.pts[   75].y = 0.136940; points.pts[   75].z = 0.025787;
    points.pts[   76].x = -0.018278; points.pts[   76].y = 0.122380; points.pts[   76].z = 0.030773;
    points.pts[   77].x = -0.021656; points.pts[   77].y = 0.116430; points.pts[   77].z = 0.035209;
    points.pts[   78].x = -0.031921; points.pts[   78].y = 0.115660; points.pts[   78].z = 0.032851;
    points.pts[   79].x = -0.021348; points.pts[   79].y = 0.124210; points.pts[   79].z = 0.024562;
    points.pts[   80].x = -0.032410; points.pts[   80].y = 0.123490; points.pts[   80].z = 0.023293;
    points.pts[   81].x = -0.024869; points.pts[   81].y = 0.120940; points.pts[   81].z = 0.028745;
    points.pts[   82].x = -0.031747; points.pts[   82].y = 0.120390; points.pts[   82].z = 0.028229;
    points.pts[   83].x = -0.052912; points.pts[   83].y = 0.126860; points.pts[   83].z = 0.034968;
    points.pts[   84].x = -0.041672; points.pts[   84].y = 0.115640; points.pts[   84].z = 0.032998;
    points.pts[   85].x = -0.052037; points.pts[   85].y = 0.116800; points.pts[   85].z = 0.034582;
    points.pts[   86].x = -0.042495; points.pts[   86].y = 0.124880; points.pts[   86].z = 0.024082;
    points.pts[   87].x = -0.047946; points.pts[   87].y = 0.127360; points.pts[   87].z = 0.028108;
    points.pts[   88].x = -0.042421; points.pts[   88].y = 0.120350; points.pts[   88].z = 0.028633;
    points.pts[   89].x = -0.047661; points.pts[   89].y = 0.120240; points.pts[   89].z = 0.028871;
    points.pts[   90].x = -0.035964; points.pts[   90].y = 0.151300; points.pts[   90].z = 0.000539;
    points.pts[   91].x = -0.050598; points.pts[   91].y = 0.147400; points.pts[   91].z = 0.013881;
    points.pts[   92].x = -0.046375; points.pts[   92].y = 0.132930; points.pts[   92].z = 0.018289;
    points.pts[   93].x = -0.049125; points.pts[   93].y = 0.138560; points.pts[   93].z = 0.016269;
    points.pts[   94].x = -0.042976; points.pts[   94].y = 0.149150; points.pts[   94].z = 0.005400;
    points.pts[   95].x = -0.047965; points.pts[   95].y = 0.146590; points.pts[   95].z = 0.008678;
    points.pts[   96].x = -0.022926; points.pts[   96].y = 0.126300; points.pts[   96].z = 0.018077;
    points.pts[   97].x = -0.031583; points.pts[   97].y = 0.125900; points.pts[   97].z = 0.017804;
    points.pts[   98].x = -0.041733; points.pts[   98].y = 0.127960; points.pts[   98].z = 0.016650;
    points.pts[   99].x = -0.061482; points.pts[   99].y = 0.146980; points.pts[   99].z = 0.036168;
    points.pts[  100].x = -0.071729; points.pts[  100].y = 0.150260; points.pts[  100].z = 0.038328;
    points.pts[  101].x = -0.060526; points.pts[  101].y = 0.136800; points.pts[  101].z = 0.035999;
    points.pts[  102].x = -0.082619; points.pts[  102].y = 0.148230; points.pts[  102].z = 0.035955;
    points.pts[  103].x = -0.087824; points.pts[  103].y = 0.144490; points.pts[  103].z = 0.033779;
    points.pts[  104].x = -0.089000; points.pts[  104].y = 0.138280; points.pts[  104].z = 0.037774;
    points.pts[  105].x = -0.085662; points.pts[  105].y = 0.150950; points.pts[  105].z = 0.028208;
    points.pts[  106].x = -0.089601; points.pts[  106].y = 0.147250; points.pts[  106].z = 0.025869;
    points.pts[  107].x = -0.090681; points.pts[  107].y = 0.137480; points.pts[  107].z = 0.023690;
    points.pts[  108].x = -0.058722; points.pts[  108].y = 0.129240; points.pts[  108].z = 0.038992;
    points.pts[  109].x = -0.060075; points.pts[  109].y = 0.115120; points.pts[  109].z = 0.037685;
    points.pts[  110].x = -0.091812; points.pts[  110].y = 0.127670; points.pts[  110].z = 0.038703;
    points.pts[  111].x = -0.091727; points.pts[  111].y = 0.116570; points.pts[  111].z = 0.039619;
    points.pts[  112].x = -0.093164; points.pts[  112].y = 0.127210; points.pts[  112].z = 0.025211;
    points.pts[  113].x = -0.093938; points.pts[  113].y = 0.120670; points.pts[  113].z = 0.024399;
    points.pts[  114].x = -0.091583; points.pts[  114].y = 0.145220; points.pts[  114].z = 0.019860;
    points.pts[  115].x = -0.090929; points.pts[  115].y = 0.136670; points.pts[  115].z = 0.019817;
    points.pts[  116].x = -0.093094; points.pts[  116].y = 0.116350; points.pts[  116].z = 0.018959;
    points.pts[  117].x = 0.024948;  points.pts[  117].y = 0.102860; points.pts[  117].z = 0.041418;
    points.pts[  118].x = 0.033600;  points.pts[  118].y = 0.092627; points.pts[  118].z = 0.040463;
    points.pts[  119].x = 0.027420;  points.pts[  119].y = 0.096386; points.pts[  119].z = 0.043312;
    points.pts[  120].x = 0.033920;  points.pts[  120].y = 0.086911; points.pts[  120].z = 0.041034;
    points.pts[  121].x = 0.028156;  points.pts[  121].y = 0.086837; points.pts[  121].z = 0.045084;
    points.pts[  122].x = 0.033810;  points.pts[  122].y = 0.078604; points.pts[  122].z = 0.040854;
    points.pts[  123].x = 0.028125;  points.pts[  123].y = 0.076874; points.pts[  123].z = 0.045059;
    points.pts[  124].x = 0.014500;  points.pts[  124].y = 0.093279; points.pts[  124].z = 0.050880;
    points.pts[  125].x = 0.007482;  points.pts[  125].y = 0.094730; points.pts[  125].z = 0.052315;
    points.pts[  126].x = 0.017407;  points.pts[  126].y = 0.105350; points.pts[  126].z = 0.043139;
    points.pts[  127].x = 0.007954;  points.pts[  127].y = 0.106330; points.pts[  127].z = 0.042968;
    points.pts[  128].x = 0.018511;  points.pts[  128].y = 0.097194; points.pts[  128].z = 0.047253;
    points.pts[  129].x = 0.008644;  points.pts[  129].y = 0.099323; points.pts[  129].z = 0.048079;
    points.pts[  130].x = -0.002020; points.pts[  130].y = 0.095698; points.pts[  130].z = 0.053906;
    points.pts[  131].x = -0.011446; points.pts[  131].y = 0.095169; points.pts[  131].z = 0.053862;
    points.pts[  132].x = -0.001875; points.pts[  132].y = 0.106910; points.pts[  132].z = 0.043455;
    points.pts[  133].x = -0.011875; points.pts[  133].y = 0.106880; points.pts[  133].z = 0.043019;
    points.pts[  134].x = -0.001762; points.pts[  134].y = 0.100710; points.pts[  134].z = 0.046648;
    points.pts[  135].x = -0.012498; points.pts[  135].y = 0.100080; points.pts[  135].z = 0.045916;
    points.pts[  136].x = 0.016381;  points.pts[  136].y = 0.085894; points.pts[  136].z = 0.051642;
    points.pts[  137].x = 0.008117;  points.pts[  137].y = 0.086910; points.pts[  137].z = 0.055228;
    points.pts[  138].x = 0.017644;  points.pts[  138].y = 0.076955; points.pts[  138].z = 0.052372;
    points.pts[  139].x = 0.008125;  points.pts[  139].y = 0.076853; points.pts[  139].z = 0.055536;
    points.pts[  140].x = 0.020575;  points.pts[  140].y = 0.088169; points.pts[  140].z = 0.049006;
    points.pts[  141].x = 0.022445;  points.pts[  141].y = 0.075721; points.pts[  141].z = 0.049563;
    points.pts[  142].x = -0.001793; points.pts[  142].y = 0.086849; points.pts[  142].z = 0.056843;
    points.pts[  143].x = -0.011943; points.pts[  143].y = 0.086771; points.pts[  143].z = 0.057009;
    points.pts[  144].x = -0.001957; points.pts[  144].y = 0.076863; points.pts[  144].z = 0.057803;
    points.pts[  145].x = -0.011875; points.pts[  145].y = 0.076964; points.pts[  145].z = 0.057022;
    points.pts[  146].x = 0.033250;  points.pts[  146].y = 0.067541; points.pts[  146].z = 0.040033;
    points.pts[  147].x = 0.028149;  points.pts[  147].y = 0.066829; points.pts[  147].z = 0.042953;
    points.pts[  148].x = 0.026761;  points.pts[  148].y = 0.057829; points.pts[  148].z = 0.042588;
    points.pts[  149].x = 0.023571;  points.pts[  149].y = 0.047460; points.pts[  149].z = 0.040428;
    points.pts[  150].x = 0.015832;  points.pts[  150].y = 0.067418; points.pts[  150].z = 0.051639;
    points.pts[  151].x = 0.008043;  points.pts[  151].y = 0.066902; points.pts[  151].z = 0.055006;
    points.pts[  152].x = 0.013984;  points.pts[  152].y = 0.058886; points.pts[  152].z = 0.050416;
    points.pts[  153].x = 0.008097;  points.pts[  153].y = 0.056888; points.pts[  153].z = 0.052950;
    points.pts[  154].x = 0.020566;  points.pts[  154].y = 0.065958; points.pts[  154].z = 0.048300;
    points.pts[  155].x = 0.018594;  points.pts[  155].y = 0.056539; points.pts[  155].z = 0.047879;
    points.pts[  156].x = 0.012875;  points.pts[  156].y = 0.052652; points.pts[  156].z = 0.049689;
    points.pts[  157].x = -0.001785; points.pts[  157].y = 0.066712; points.pts[  157].z = 0.056503;
    points.pts[  158].x = -0.011785; points.pts[  158].y = 0.066885; points.pts[  158].z = 0.055015;
    points.pts[  159].x = -0.001875; points.pts[  159].y = 0.056597; points.pts[  159].z = 0.054410;
    points.pts[  160].x = -0.011840; points.pts[  160].y = 0.057054; points.pts[  160].z = 0.052714;
    points.pts[  161].x = -0.015688; points.pts[  161].y = 0.052469; points.pts[  161].z = 0.049615;
    points.pts[  162].x = 0.006615;  points.pts[  162].y = 0.049930; points.pts[  162].z = 0.051259;
    points.pts[  163].x = 0.018088;  points.pts[  163].y = 0.046655; points.pts[  163].z = 0.043321;
    points.pts[  164].x = 0.008841;  points.pts[  164].y = 0.045437; points.pts[  164].z = 0.046623;
    points.pts[  165].x = 0.017688;  points.pts[  165].y = 0.039719; points.pts[  165].z = 0.043084;
    points.pts[  166].x = 0.008125;  points.pts[  166].y = 0.039516; points.pts[  166].z = 0.045374;
    points.pts[  167].x = -0.001611; points.pts[  167].y = 0.049844; points.pts[  167].z = 0.051720;
    points.pts[  168].x = -0.012450; points.pts[  168].y = 0.046773; points.pts[  168].z = 0.050903;
    points.pts[  169].x = -0.013851; points.pts[  169].y = 0.039778; points.pts[  169].z = 0.051036;
    points.pts[  170].x = -0.002029; points.pts[  170].y = 0.044874; points.pts[  170].z = 0.047587;
    points.pts[  171].x = -0.011653; points.pts[  171].y = 0.046860; points.pts[  171].z = 0.048661;
    points.pts[  172].x = -0.001861; points.pts[  172].y = 0.039606; points.pts[  172].z = 0.047339;
    points.pts[  173].x = -0.009155; points.pts[  173].y = 0.039580; points.pts[  173].z = 0.049415;
    points.pts[  174].x = 0.043661;  points.pts[  174].y = 0.094028; points.pts[  174].z = 0.022520;
    points.pts[  175].x = 0.034642;  points.pts[  175].y = 0.104730; points.pts[  175].z = 0.031831;
    points.pts[  176].x = 0.028343;  points.pts[  176].y = 0.107200; points.pts[  176].z = 0.036339;
    points.pts[  177].x = 0.036339;  points.pts[  177].y = 0.096552; points.pts[  177].z = 0.034843;
    points.pts[  178].x = 0.031733;  points.pts[  178].y = 0.099372; points.pts[  178].z = 0.038505;
    points.pts[  179].x = 0.036998;  points.pts[  179].y = 0.106680; points.pts[  179].z = 0.026781;
    points.pts[  180].x = 0.032875;  points.pts[  180].y = 0.111080; points.pts[  180].z = 0.029590;
    points.pts[  181].x = 0.040938;  points.pts[  181].y = 0.097132; points.pts[  181].z = 0.026663;
    points.pts[  182].x = 0.044153;  points.pts[  182].y = 0.086466; points.pts[  182].z = 0.024241;
    points.pts[  183].x = 0.053750;  points.pts[  183].y = 0.072221; points.pts[  183].z = 0.020429;
    points.pts[  184].x = 0.045160;  points.pts[  184].y = 0.076574; points.pts[  184].z = 0.023594;
    points.pts[  185].x = 0.038036;  points.pts[  185].y = 0.086663; points.pts[  185].z = 0.035459;
    points.pts[  186].x = 0.037861;  points.pts[  186].y = 0.076625; points.pts[  186].z = 0.035658;
    points.pts[  187].x = 0.042216;  points.pts[  187].y = 0.087237; points.pts[  187].z = 0.028254;
    points.pts[  188].x = 0.042355;  points.pts[  188].y = 0.076747; points.pts[  188].z = 0.028580;
    points.pts[  189].x = 0.043875;  points.pts[  189].y = 0.096228; points.pts[  189].z = 0.015269;
    points.pts[  190].x = 0.044375;  points.pts[  190].y = 0.096797; points.pts[  190].z = 0.008644;
    points.pts[  191].x = 0.039545;  points.pts[  191].y = 0.106100; points.pts[  191].z = 0.017655;
    points.pts[  192].x = 0.042313;  points.pts[  192].y = 0.100090; points.pts[  192].z = 0.017237;
    points.pts[  193].x = 0.045406;  points.pts[  193].y = 0.087417; points.pts[  193].z = 0.015604;
    points.pts[  194].x = 0.055118;  points.pts[  194].y = 0.072639; points.pts[  194].z = 0.017944;
    points.pts[  195].x = 0.048722;  points.pts[  195].y = 0.073760; points.pts[  195].z = 0.017434;
    points.pts[  196].x = 0.045917;  points.pts[  196].y = 0.086298; points.pts[  196].z = 0.009421;
    points.pts[  197].x = 0.019433;  points.pts[  197].y = 0.109600; points.pts[  197].z = 0.039063;
    points.pts[  198].x = 0.010970;  points.pts[  198].y = 0.110580; points.pts[  198].z = 0.039648;
    points.pts[  199].x = 0.046657;  points.pts[  199].y = 0.057153; points.pts[  199].z = 0.031337;
    points.pts[  200].x = 0.056079;  points.pts[  200].y = 0.066335; points.pts[  200].z = 0.024122;
    points.pts[  201].x = 0.048168;  points.pts[  201].y = 0.067010; points.pts[  201].z = 0.026298;
    points.pts[  202].x = 0.056055;  points.pts[  202].y = 0.057253; points.pts[  202].z = 0.024902;
    points.pts[  203].x = 0.051163;  points.pts[  203].y = 0.056662; points.pts[  203].z = 0.029137;
    points.pts[  204].x = 0.036914;  points.pts[  204].y = 0.067032; points.pts[  204].z = 0.036122;
    points.pts[  205].x = 0.033000;  points.pts[  205].y = 0.064720; points.pts[  205].z = 0.039903;
    points.pts[  206].x = 0.038004;  points.pts[  206].y = 0.056507; points.pts[  206].z = 0.033119;
    points.pts[  207].x = 0.030629;  points.pts[  207].y = 0.054915; points.pts[  207].z = 0.038484;
    points.pts[  208].x = 0.041875;  points.pts[  208].y = 0.066383; points.pts[  208].z = 0.028357;
    points.pts[  209].x = 0.041434;  points.pts[  209].y = 0.060880; points.pts[  209].z = 0.029632;
    points.pts[  210].x = 0.044921;  points.pts[  210].y = 0.049904; points.pts[  210].z = 0.031243;
    points.pts[  211].x = 0.054635;  points.pts[  211].y = 0.050167; points.pts[  211].z = 0.022044;
    points.pts[  212].x = 0.048280;  points.pts[  212].y = 0.047370; points.pts[  212].z = 0.025845;
    points.pts[  213].x = 0.037973;  points.pts[  213].y = 0.048347; points.pts[  213].z = 0.031456;
    points.pts[  214].x = 0.028053;  points.pts[  214].y = 0.047061; points.pts[  214].z = 0.035991;
    points.pts[  215].x = 0.025595;  points.pts[  215].y = 0.040346; points.pts[  215].z = 0.034150;
    points.pts[  216].x = 0.038455;  points.pts[  216].y = 0.043509; points.pts[  216].z = 0.028278;
    points.pts[  217].x = 0.032031;  points.pts[  217].y = 0.043278; points.pts[  217].z = 0.029253;
    points.pts[  218].x = 0.036581;  points.pts[  218].y = 0.040335; points.pts[  218].z = 0.025144;
    points.pts[  219].x = 0.030190;  points.pts[  219].y = 0.039321; points.pts[  219].z = 0.026847;
    points.pts[  220].x = 0.059333;  points.pts[  220].y = 0.067891; points.pts[  220].z = 0.017361;
    points.pts[  221].x = 0.046500;  points.pts[  221].y = 0.071452; points.pts[  221].z = 0.019710;
    points.pts[  222].x = 0.059562;  points.pts[  222].y = 0.057747; points.pts[  222].z = 0.018340;
    points.pts[  223].x = 0.055636;  points.pts[  223].y = 0.049199; points.pts[  223].z = 0.019173;
    points.pts[  224].x = 0.050500;  points.pts[  224].y = 0.045064; points.pts[  224].z = 0.019181;
    points.pts[  225].x = 0.023000;  points.pts[  225].y = 0.047803; points.pts[  225].z = 0.039776;
    points.pts[  226].x = 0.022389;  points.pts[  226].y = 0.038860; points.pts[  226].z = 0.038795;
    points.pts[  227].x = -0.019545; points.pts[  227].y = 0.093900; points.pts[  227].z = 0.052205;
    points.pts[  228].x = -0.021462; points.pts[  228].y = 0.106180; points.pts[  228].z = 0.042059;
    points.pts[  229].x = -0.031027; points.pts[  229].y = 0.103950; points.pts[  229].z = 0.041228;
    points.pts[  230].x = -0.022521; points.pts[  230].y = 0.097723; points.pts[  230].z = 0.045194;
    points.pts[  231].x = -0.031858; points.pts[  231].y = 0.097026; points.pts[  231].z = 0.043878;
    points.pts[  232].x = -0.043262; points.pts[  232].y = 0.104120; points.pts[  232].z = 0.040891;
    points.pts[  233].x = -0.052154; points.pts[  233].y = 0.104040; points.pts[  233].z = 0.040972;
    points.pts[  234].x = -0.041875; points.pts[  234].y = 0.096944; points.pts[  234].z = 0.042424;
    points.pts[  235].x = -0.051919; points.pts[  235].y = 0.096967; points.pts[  235].z = 0.043563;
    points.pts[  236].x = -0.021489; points.pts[  236].y = 0.086672; points.pts[  236].z = 0.054767;
    points.pts[  237].x = -0.027000; points.pts[  237].y = 0.083087; points.pts[  237].z = 0.050284;
    points.pts[  238].x = -0.021070; points.pts[  238].y = 0.077249; points.pts[  238].z = 0.054365;
    points.pts[  239].x = -0.026011; points.pts[  239].y = 0.089634; points.pts[  239].z = 0.048981;
    points.pts[  240].x = -0.031893; points.pts[  240].y = 0.087035; points.pts[  240].z = 0.044169;
    points.pts[  241].x = -0.025625; points.pts[  241].y = 0.074892; points.pts[  241].z = 0.047102;
    points.pts[  242].x = -0.031970; points.pts[  242].y = 0.076900; points.pts[  242].z = 0.042177;
    points.pts[  243].x = -0.041824; points.pts[  243].y = 0.086954; points.pts[  243].z = 0.043295;
    points.pts[  244].x = -0.051825; points.pts[  244].y = 0.086844; points.pts[  244].z = 0.044933;
    points.pts[  245].x = -0.041918; points.pts[  245].y = 0.076728; points.pts[  245].z = 0.042564;
    points.pts[  246].x = -0.051849; points.pts[  246].y = 0.076877; points.pts[  246].z = 0.042992;
    points.pts[  247].x = -0.061339; points.pts[  247].y = 0.103930; points.pts[  247].z = 0.041164;
    points.pts[  248].x = -0.072672; points.pts[  248].y = 0.109760; points.pts[  248].z = 0.044294;
    points.pts[  249].x = -0.061784; points.pts[  249].y = 0.096825; points.pts[  249].z = 0.043327;
    points.pts[  250].x = -0.070058; points.pts[  250].y = 0.096203; points.pts[  250].z = 0.041397;
    points.pts[  251].x = -0.080439; points.pts[  251].y = 0.110910; points.pts[  251].z = 0.044343;
    points.pts[  252].x = -0.061927; points.pts[  252].y = 0.086724; points.pts[  252].z = 0.044520;
    points.pts[  253].x = -0.070344; points.pts[  253].y = 0.087352; points.pts[  253].z = 0.041908;
    points.pts[  254].x = -0.061410; points.pts[  254].y = 0.077489; points.pts[  254].z = 0.042178;
    points.pts[  255].x = -0.068579; points.pts[  255].y = 0.080144; points.pts[  255].z = 0.041024;
    points.pts[  256].x = -0.019045; points.pts[  256].y = 0.067732; points.pts[  256].z = 0.052388;
    points.pts[  257].x = -0.017742; points.pts[  257].y = 0.058909; points.pts[  257].z = 0.050809;
    points.pts[  258].x = -0.023548; points.pts[  258].y = 0.066382; points.pts[  258].z = 0.045226;
    points.pts[  259].x = -0.033990; points.pts[  259].y = 0.067795; points.pts[  259].z = 0.040929;
    points.pts[  260].x = -0.021690; points.pts[  260].y = 0.056549; points.pts[  260].z = 0.045164;
    points.pts[  261].x = -0.036111; points.pts[  261].y = 0.060706; points.pts[  261].z = 0.040407;
    points.pts[  262].x = -0.041231; points.pts[  262].y = 0.066951; points.pts[  262].z = 0.041392;
    points.pts[  263].x = -0.048588; points.pts[  263].y = 0.070956; points.pts[  263].z = 0.040357;
    points.pts[  264].x = -0.040300; points.pts[  264].y = 0.059465; points.pts[  264].z = 0.040446;
    points.pts[  265].x = -0.021920; points.pts[  265].y = 0.044965; points.pts[  265].z = 0.052258;
    points.pts[  266].x = -0.029187; points.pts[  266].y = 0.043585; points.pts[  266].z = 0.051088;
    points.pts[  267].x = -0.021919; points.pts[  267].y = 0.039826; points.pts[  267].z = 0.053521;
    points.pts[  268].x = -0.030331; points.pts[  268].y = 0.039749; points.pts[  268].z = 0.052133;
    points.pts[  269].x = -0.021998; points.pts[  269].y = 0.049847; points.pts[  269].z = 0.046725;
    points.pts[  270].x = -0.031911; points.pts[  270].y = 0.046848; points.pts[  270].z = 0.045187;
    points.pts[  271].x = -0.035276; points.pts[  271].y = 0.039753; points.pts[  271].z = 0.047529;
    points.pts[  272].x = -0.042016; points.pts[  272].y = 0.044823; points.pts[  272].z = 0.041594;
    points.pts[  273].x = -0.051940; points.pts[  273].y = 0.044707; points.pts[  273].z = 0.043498;
    points.pts[  274].x = -0.041928; points.pts[  274].y = 0.039327; points.pts[  274].z = 0.043582;
    points.pts[  275].x = -0.051857; points.pts[  275].y = 0.039252; points.pts[  275].z = 0.046212;
    points.pts[  276].x = -0.059453; points.pts[  276].y = 0.044240; points.pts[  276].z = 0.042862;
    points.pts[  277].x = -0.060765; points.pts[  277].y = 0.039087; points.pts[  277].z = 0.044363;
    points.pts[  278].x = -0.024273; points.pts[  278].y = 0.110380; points.pts[  278].z = 0.039129;
    points.pts[  279].x = -0.032379; points.pts[  279].y = 0.108780; points.pts[  279].z = 0.037952;
    points.pts[  280].x = -0.041152; points.pts[  280].y = 0.108530; points.pts[  280].z = 0.037969;
    points.pts[  281].x = -0.051698; points.pts[  281].y = 0.109060; points.pts[  281].z = 0.038258;
    points.pts[  282].x = -0.062091; points.pts[  282].y = 0.108770; points.pts[  282].z = 0.038274;
    points.pts[  283].x = -0.071655; points.pts[  283].y = 0.105960; points.pts[  283].z = 0.037516;
    points.pts[  284].x = -0.074634; points.pts[  284].y = 0.097746; points.pts[  284].z = 0.038347;
    points.pts[  285].x = -0.079120; points.pts[  285].y = 0.105080; points.pts[  285].z = 0.032308;
    points.pts[  286].x = -0.080203; points.pts[  286].y = 0.096758; points.pts[  286].z = 0.033592;
    points.pts[  287].x = -0.083780; points.pts[  287].y = 0.105680; points.pts[  287].z = 0.025985;
    points.pts[  288].x = -0.087292; points.pts[  288].y = 0.103140; points.pts[  288].z = 0.020825;
    points.pts[  289].x = -0.085210; points.pts[  289].y = 0.097079; points.pts[  289].z = 0.027810;
    points.pts[  290].x = -0.088082; points.pts[  290].y = 0.096456; points.pts[  290].z = 0.022985;
    points.pts[  291].x = -0.075160; points.pts[  291].y = 0.086040; points.pts[  291].z = 0.038816;
    points.pts[  292].x = -0.064577; points.pts[  292].y = 0.073455; points.pts[  292].z = 0.038970;
    points.pts[  293].x = -0.072279; points.pts[  293].y = 0.076416; points.pts[  293].z = 0.036413;
    points.pts[  294].x = -0.076375; points.pts[  294].y = 0.072563; points.pts[  294].z = 0.028730;
    points.pts[  295].x = -0.080031; points.pts[  295].y = 0.087076; points.pts[  295].z = 0.034290;
    points.pts[  296].x = -0.078919; points.pts[  296].y = 0.079371; points.pts[  296].z = 0.032477;
    points.pts[  297].x = -0.084834; points.pts[  297].y = 0.086686; points.pts[  297].z = 0.026974;
    points.pts[  298].x = -0.087891; points.pts[  298].y = 0.089233; points.pts[  298].z = 0.022611;
    points.pts[  299].x = -0.081048; points.pts[  299].y = 0.077169; points.pts[  299].z = 0.025829;
    points.pts[  300].x = -0.086393; points.pts[  300].y = 0.107840; points.pts[  300].z = 0.018635;
    points.pts[  301].x = -0.087672; points.pts[  301].y = 0.104920; points.pts[  301].z = 0.017264;
    points.pts[  302].x = -0.089333; points.pts[  302].y = 0.098483; points.pts[  302].z = 0.017610;
    points.pts[  303].x = -0.086375; points.pts[  303].y = 0.083067; points.pts[  303].z = 0.018607;
    points.pts[  304].x = -0.089179; points.pts[  304].y = 0.089186; points.pts[  304].z = 0.018947;
    points.pts[  305].x = -0.082879; points.pts[  305].y = 0.076109; points.pts[  305].z = 0.017794;
    points.pts[  306].x = -0.082500; points.pts[  306].y = 0.074674; points.pts[  306].z = 0.007118;
    points.pts[  307].x = -0.026437; points.pts[  307].y = 0.064141; points.pts[  307].z = 0.039321;
    points.pts[  308].x = -0.030035; points.pts[  308].y = 0.066130; points.pts[  308].z = 0.038942;
    points.pts[  309].x = -0.026131; points.pts[  309].y = 0.056531; points.pts[  309].z = 0.038882;
    points.pts[  310].x = -0.031664; points.pts[  310].y = 0.056657; points.pts[  310].z = 0.037742;
    points.pts[  311].x = -0.045716; points.pts[  311].y = 0.064541; points.pts[  311].z = 0.039166;
    points.pts[  312].x = -0.051959; points.pts[  312].y = 0.066869; points.pts[  312].z = 0.036733;
    points.pts[  313].x = -0.042557; points.pts[  313].y = 0.055545; points.pts[  313].z = 0.039026;
    points.pts[  314].x = -0.049406; points.pts[  314].y = 0.056892; points.pts[  314].z = 0.034344;
    points.pts[  315].x = -0.055500; points.pts[  315].y = 0.062391; points.pts[  315].z = 0.029498;
    points.pts[  316].x = -0.053750; points.pts[  316].y = 0.058574; points.pts[  316].z = 0.026313;
    points.pts[  317].x = -0.034060; points.pts[  317].y = 0.050137; points.pts[  317].z = 0.038577;
    points.pts[  318].x = -0.041741; points.pts[  318].y = 0.049590; points.pts[  318].z = 0.039290;
    points.pts[  319].x = -0.050975; points.pts[  319].y = 0.049435; points.pts[  319].z = 0.036965;
    points.pts[  320].x = -0.053000; points.pts[  320].y = 0.051065; points.pts[  320].z = 0.029209;
    points.pts[  321].x = -0.054145; points.pts[  321].y = 0.054568; points.pts[  321].z = 0.012257;
    points.pts[  322].x = -0.055848; points.pts[  322].y = 0.054170; points.pts[  322].z = 0.008327;
    points.pts[  323].x = -0.054844; points.pts[  323].y = 0.049295; points.pts[  323].z = 0.011462;
    points.pts[  324].x = -0.056150; points.pts[  324].y = 0.050619; points.pts[  324].z = 0.009293;
    points.pts[  325].x = -0.061451; points.pts[  325].y = 0.068257; points.pts[  325].z = 0.035376;
    points.pts[  326].x = -0.069725; points.pts[  326].y = 0.069958; points.pts[  326].z = 0.032788;
    points.pts[  327].x = -0.062823; points.pts[  327].y = 0.063322; points.pts[  327].z = 0.026886;
    points.pts[  328].x = -0.071037; points.pts[  328].y = 0.066787; points.pts[  328].z = 0.025228;
    points.pts[  329].x = -0.060857; points.pts[  329].y = 0.060568; points.pts[  329].z = 0.022643;
    points.pts[  330].x = -0.067000; points.pts[  330].y = 0.061558; points.pts[  330].z = 0.020109;
    points.pts[  331].x = -0.078200; points.pts[  331].y = 0.071279; points.pts[  331].z = 0.021032;
    points.pts[  332].x = -0.062116; points.pts[  332].y = 0.045145; points.pts[  332].z = 0.037802;
    points.pts[  333].x = -0.065473; points.pts[  333].y = 0.039513; points.pts[  333].z = 0.037964;
    points.pts[  334].x = -0.067250; points.pts[  334].y = 0.037420; points.pts[  334].z = 0.033413;
    points.pts[  335].x = -0.072702; points.pts[  335].y = 0.065008; points.pts[  335].z = 0.018701;
    points.pts[  336].x = -0.061450; points.pts[  336].y = 0.059165; points.pts[  336].z = 0.018731;
    points.pts[  337].x = -0.067500; points.pts[  337].y = 0.061479; points.pts[  337].z = 0.019221;
    points.pts[  338].x = -0.057411; points.pts[  338].y = 0.054114; points.pts[  338].z = 0.003826;
    points.pts[  339].x = -0.079222; points.pts[  339].y = 0.070654; points.pts[  339].z = 0.017735;
    points.pts[  340].x = -0.062473; points.pts[  340].y = 0.044680; points.pts[  340].z = 0.011110;
    points.pts[  341].x = -0.067250; points.pts[  341].y = 0.042258; points.pts[  341].z = 0.010414;
    points.pts[  342].x = -0.066389; points.pts[  342].y = 0.040515; points.pts[  342].z = 0.013160;
    points.pts[  343].x = -0.068359; points.pts[  343].y = 0.038502; points.pts[  343].z = 0.011958;
    points.pts[  344].x = -0.061381; points.pts[  344].y = 0.047480; points.pts[  344].z = 0.007607;
    points.pts[  345].x = -0.068559; points.pts[  345].y = 0.043549; points.pts[  345].z = 0.008158;
    points.pts[  346].x = -0.070929; points.pts[  346].y = 0.039830; points.pts[  346].z = 0.008589;
    points.pts[  347].x = -0.016625; points.pts[  347].y = 0.183750; points.pts[  347].z = -0.019735;
    points.pts[  348].x = -0.015198; points.pts[  348].y = 0.174710; points.pts[  348].z = -0.018868;
    points.pts[  349].x = -0.015944; points.pts[  349].y = 0.162640; points.pts[  349].z = -0.009104;
    points.pts[  350].x = -0.015977; points.pts[  350].y = 0.160700; points.pts[  350].z = -0.008807;
    points.pts[  351].x = -0.013251; points.pts[  351].y = 0.167080; points.pts[  351].z = -0.015264;
    points.pts[  352].x = -0.014292; points.pts[  352].y = 0.160980; points.pts[  352].z = -0.011252;
    points.pts[  353].x = -0.013986; points.pts[  353].y = 0.184000; points.pts[  353].z = -0.023739;
    points.pts[  354].x = -0.011633; points.pts[  354].y = 0.176990; points.pts[  354].z = -0.023349;
    points.pts[  355].x = -0.009103; points.pts[  355].y = 0.169880; points.pts[  355].z = -0.021457;
    points.pts[  356].x = -0.025562; points.pts[  356].y = 0.182730; points.pts[  356].z = -0.009625;
    points.pts[  357].x = -0.027250; points.pts[  357].y = 0.182540; points.pts[  357].z = -0.009438;
    points.pts[  358].x = -0.025736; points.pts[  358].y = 0.179480; points.pts[  358].z = -0.008965;
    points.pts[  359].x = -0.031216; points.pts[  359].y = 0.175890; points.pts[  359].z = -0.005115;
    points.pts[  360].x = -0.020399; points.pts[  360].y = 0.184500; points.pts[  360].z = -0.014943;
    points.pts[  361].x = -0.021339; points.pts[  361].y = 0.176450; points.pts[  361].z = -0.014566;
    points.pts[  362].x = -0.027125; points.pts[  362].y = 0.172340; points.pts[  362].z = -0.010156;
    points.pts[  363].x = -0.039390; points.pts[  363].y = 0.173300; points.pts[  363].z = -0.002357;
    points.pts[  364].x = -0.022876; points.pts[  364].y = 0.164060; points.pts[  364].z = -0.007810;
    points.pts[  365].x = -0.031597; points.pts[  365].y = 0.166510; points.pts[  365].z = -0.004929;
    points.pts[  366].x = -0.022600; points.pts[  366].y = 0.159120; points.pts[  366].z = -0.003799;
    points.pts[  367].x = -0.030372; points.pts[  367].y = 0.157670; points.pts[  367].z = -0.001267;
    points.pts[  368].x = -0.021158; points.pts[  368].y = 0.168490; points.pts[  368].z = -0.012383;
    points.pts[  369].x = -0.027000; points.pts[  369].y = 0.171200; points.pts[  369].z = -0.010220;
    points.pts[  370].x = -0.041719; points.pts[  370].y = 0.168130; points.pts[  370].z = -0.000750;
    points.pts[  371].x = -0.048250; points.pts[  371].y = 0.167480; points.pts[  371].z = -0.000152;
    points.pts[  372].x = -0.037250; points.pts[  372].y = 0.161470; points.pts[  372].z = -0.000073;
    points.pts[  373].x = -0.066429; points.pts[  373].y = 0.157830; points.pts[  373].z = -0.008567;
    points.pts[  374].x = -0.071284; points.pts[  374].y = 0.158390; points.pts[  374].z = -0.005998;
    points.pts[  375].x = -0.065979; points.pts[  375].y = 0.162880; points.pts[  375].z = -0.017792;
    points.pts[  376].x = -0.071623; points.pts[  376].y = 0.163840; points.pts[  376].z = -0.015760;
    points.pts[  377].x = -0.066068; points.pts[  377].y = 0.160510; points.pts[  377].z = -0.013567;
    points.pts[  378].x = -0.073307; points.pts[  378].y = 0.160490; points.pts[  378].z = -0.011832;
    points.pts[  379].x = -0.077000; points.pts[  379].y = 0.162040; points.pts[  379].z = -0.019241;
    points.pts[  380].x = -0.077179; points.pts[  380].y = 0.158510; points.pts[  380].z = -0.014950;
    points.pts[  381].x = -0.073691; points.pts[  381].y = 0.172860; points.pts[  381].z = -0.037944;
    points.pts[  382].x = -0.077550; points.pts[  382].y = 0.172210; points.pts[  382].z = -0.039175;
    points.pts[  383].x = -0.065921; points.pts[  383].y = 0.165860; points.pts[  383].z = -0.025022;
    points.pts[  384].x = -0.072095; points.pts[  384].y = 0.167840; points.pts[  384].z = -0.024725;
    points.pts[  385].x = -0.066000; points.pts[  385].y = 0.168080; points.pts[  385].z = -0.030916;
    points.pts[  386].x = -0.073448; points.pts[  386].y = 0.170510; points.pts[  386].z = -0.032045;
    points.pts[  387].x = -0.077770; points.pts[  387].y = 0.164340; points.pts[  387].z = -0.025938;
    points.pts[  388].x = -0.077893; points.pts[  388].y = 0.160390; points.pts[  388].z = -0.021299;
    points.pts[  389].x = -0.078211; points.pts[  389].y = 0.169000; points.pts[  389].z = -0.034566;
    points.pts[  390].x = -0.034667; points.pts[  390].y = 0.151310; points.pts[  390].z = -0.000710;
    points.pts[  391].x = -0.066117; points.pts[  391].y = 0.173530; points.pts[  391].z = -0.047453;
    points.pts[  392].x = -0.071986; points.pts[  392].y = 0.176120; points.pts[  392].z = -0.045384;
    points.pts[  393].x = -0.069250; points.pts[  393].y = 0.182000; points.pts[  393].z = -0.055026;
    points.pts[  394].x = -0.064992; points.pts[  394].y = 0.178020; points.pts[  394].z = -0.054645;
    points.pts[  395].x = -0.069935; points.pts[  395].y = 0.179830; points.pts[  395].z = -0.051988;
    points.pts[  396].x = -0.077930; points.pts[  396].y = 0.175160; points.pts[  396].z = -0.044400;
  }
}

#endif
