#include "define.h"

TRANSINFO	calibInfo;

//HANDLE	dfp=INVALID_HANDLE_VALUE;
FILE    *dfp;
int		dsbytesiz = sizeof (point3d)*2 + sizeof (ONEVDNDATA);
int		dFrmNum=0;
int		dFrmNo=0;

RMAP	rm;
DMAP	dm;
DMAP	gm, ggm;

ONEDSVFRAME	*onefrm;

bool LoadCalibFile (char *szFile)
{
	char			i_line[200];
    FILE			*fp;
	MATRIX			rt;

	fp = fopen (szFile, "r");
	if (!fp) 
		return false;

	rMatrixInit (calibInfo.rot);

	int	i = 0;
	while (1) {
		if (fgets (i_line, 80, fp) == NULL)
			break;

        if (strncmp(i_line, "rot", 3) == 0) {
			strtok (i_line, " ,\t\n");
			calibInfo.ang.x = atof (strtok (NULL, " ,\t\n"))*topi;
			calibInfo.ang.y = atof (strtok (NULL, " ,\t\n"))*topi;
			calibInfo.ang.z = atof (strtok (NULL, " ,\t\n"))*topi;
			createRotMatrix_ZYX (rt, calibInfo.ang.x, calibInfo.ang.y, calibInfo.ang.z);
			rMatrixmulti (calibInfo.rot, rt);
			continue;
		}

        if (strncmp (i_line, "shv", 3) == 0) {
			strtok (i_line, " ,\t\n");
			calibInfo.shv.x = atof (strtok (NULL, " ,\t\n"));
			calibInfo.shv.y = atof (strtok (NULL, " ,\t\n"));
			calibInfo.shv.z = atof (strtok (NULL, " ,\t\n"));
		}
	}
	fclose (fp);

	return true;
}

void SmoothingData ()
{
	int maxcnt = 3;

	for (int y=0; y<rm.len; y++) {
		for (int x=1; x<(rm.wid-1); x++) {
			if (rm.pts[y*rm.wid+(x-1)].i && !rm.pts[y*rm.wid+x].i) {

				int xx;
				for (xx=x+1; xx<rm.wid; xx++) {
					if (rm.pts[y*rm.wid+xx].i)
						break;
				}
				if (xx>=rm.wid)
					continue;
				int cnt = xx-x+1;
				if (cnt>maxcnt) {
					x = xx;
					continue;
				}
				point3fi *p1 = &rm.pts[y*rm.wid+(x-1)];
				point3fi *p2 = &rm.pts[y*rm.wid+xx];
				double dis = ppDistance3fi (p1, p2);
				double rng = max(p2r(p1),p2r(p2));
				double maxdis = min(MAXSMOOTHERR, max (BASEERROR, HORIERRFACTOR*cnt*rng));
				if (dis<maxdis) {
					for (int xxx=x; xxx<xx; xxx++) {
						point3fi *p = &rm.pts[y*rm.wid+xxx];
						p->x = (p2->x-p1->x)/cnt*(xxx-x+1)+p1->x;
						p->y = (p2->y-p1->y)/cnt*(xxx-x+1)+p1->y;
						p->z = (p2->z-p1->z)/cnt*(xxx-x+1)+p1->z;
						p->i = 1;
					}
				}
				x = xx;
			}
		}
	}
}

void CorrectPoints ()
{
	MAT2D	rot1, rot2;

	//transform points to the vehicle frame of onefrm->dsv[0]
	//src: block i; tar: block 0

	//rot2: R_tar^{-1}
	rot2[0][0] = cos (-onefrm->dsv[0].ang.z);
	rot2[0][1] = -sin (-onefrm->dsv[0].ang.z);
	rot2[1][0] = sin (-onefrm->dsv[0].ang.z);
	rot2[1][1] = cos (-onefrm->dsv[0].ang.z);

	for (int i=1; i<BKNUM_PER_FRM; i++) {
		for (int j=0; j<PTNUM_PER_BLK; j++) {
			if (!onefrm->dsv[i].points[j].i)
				continue;

			rotatePoint3fi(onefrm->dsv[i].points[j], calibInfo.rot);
			shiftPoint3fi(onefrm->dsv[i].points[j], calibInfo.shv); 
			rotatePoint3fi(onefrm->dsv[i].points[j], onefrm->dsv[i].rot);

			//rot1: R_tar^{-1}*R_src
			rot1[0][0] = cos (onefrm->dsv[i].ang.z-onefrm->dsv[0].ang.z);
			rot1[0][1] = -sin (onefrm->dsv[i].ang.z-onefrm->dsv[0].ang.z);
			rot1[1][0] = sin (onefrm->dsv[i].ang.z-onefrm->dsv[0].ang.z);
			rot1[1][1] = cos (onefrm->dsv[i].ang.z-onefrm->dsv[0].ang.z);

			//shv: SHV_src-SHV_tar
			point2d shv;
			shv.x = onefrm->dsv[i].shv.x-onefrm->dsv[0].shv.x;
			shv.y = onefrm->dsv[i].shv.y-onefrm->dsv[0].shv.y;

			point2d pp;
			pp.x = onefrm->dsv[i].points[j].x; pp.y = onefrm->dsv[i].points[j].y;
			rotatePoint2d (pp, rot1);	//R_tar^{-1}*R_src*p
			rotatePoint2d (shv, rot2);	//R_tar^{-1}*(SHV_src-SHV_tar)
			shiftPoint2d (pp, shv);		//p'=R_tar^{-1}*R_src*p+R_tar^{-1}*(SHV_src-SHV_tar)
			onefrm->dsv[i].points[j].x = pp.x;
			onefrm->dsv[i].points[j].y = pp.y;
		}
	}

	for (int ry=0; ry<rm.len; ry++) {
		for (int rx=0; rx<rm.wid; rx++) {
			int i=rm.idx[ry*rm.wid+rx].x;
			int j=rm.idx[ry*rm.wid+rx].y;
			if (!i&&!j)
				continue;
			rm.pts[ry*rm.wid+rx] = onefrm->dsv[i].points[j];
		}
	}
}

void ProcessOneFrame ()
{
	//生成距离图像帧
	GenerateRangeView ();

	//根据calib参数将激光点转换到车体坐标系，根据车体角度roll、pitch修正激光帧到水平，数据点转换到第0个数据包航向角和位移所对应的车体坐标系
	CorrectPoints ();	

	//对每一行数据中短暂无效激光点（cnt<5，约水平1度)进行内插补齐，否则这些无效点处会被认为是边界点
	SmoothingData ();

    //分割出路面区域
	//第一步：标注边界点ContourExtraction();
	//第二步：区域增长方式标注区域内点RegionGrow()
	memset (rm.regionID, 0, sizeof(int)*rm.wid*rm.len);
	rm.regnum = 0;
	ContourSegger ();
	
	//为每个区域生成一个segbuf，用于分类、目前仅提取了少量特征
	if (rm.regnum) {
		rm.segbuf = new SEGBUF[rm.regnum];
		memset (rm.segbuf, 0, sizeof (SEGBUF)*rm.regnum);
        Region2Seg ();
	}	
	//生成可视化距离图像处理结果
    DrawRangeView ();
	
	//将全局DEM转换到当前车体坐标系下
	PredictGloDem (gm,ggm);

	//生成单帧数据的DEM
	GenerateLocDem (dm);

	//用当前帧DEM更新全局DEM
	UpdateGloDem (gm,dm);

    //提取道路中心线
    ExtractRoadCenterline (gm);

    //计算地面俯仰和横滚角，分类地形（上下坡）(只给可通行区域打标签)
    LabelRoadSurface (gm);

    //提取路面上的障碍物（凹凸障碍）
    LabelObstacle (gm);

	//生成可视化单帧数据DEM
	DrawDem (dm);

	//生成可视化全局DEM
	DrawDem (gm);

	if (rm.segbuf)
		delete []rm.segbuf;

}

//读取一帧vel64数据（一帧为580×12×32个激光点）保存到onefrm->dsv，未作坐标转换
BOOL ReadOneDsvFrame ()
{
	DWORD	dwReadBytes;
	int		i;

    for (i=0; i<BKNUM_PER_FRM; i++) {
        dwReadBytes = fread((ONEDSVDATA *)&onefrm->dsv[i], 1, dsbytesiz, dfp);
        if ((dsbytesiz != dwReadBytes) || (ferror(dfp))) {
            printf("Error from reading file.\n");
			break;
        }

//		createRotMatrix_ZYX(onefrm->dsv[i].rot, onefrm->dsv[i].ang.x, onefrm->dsv[i].ang.y , onefrm->dsv[i].ang.z ) ; 
		createRotMatrix_ZYX(onefrm->dsv[i].rot, onefrm->dsv[i].ang.x, onefrm->dsv[i].ang.y , 0 ) ; 
	}

	if (i<BKNUM_PER_FRM)
        return false;
	else
        return true;
}

void CallbackLocDem(int event, int x, int y, int flags, void *ustc)
{
    static CvPoint lu, rb;

    if (event == CV_EVENT_LBUTTONDOWN) {
        lu.x = x; lu.y = y;
    }
    if (event == CV_EVENT_LBUTTONUP) {

        rb.x = x; rb.y = y;
        IplImage *tmp = cvCreateImage (cvSize (dm.wid, dm.len),IPL_DEPTH_8U,3);
        cvCopy (dm.lmap, tmp);
        cvRectangle (dm.lmap, lu, rb, cvScalar(255, 255, 0), 3);
        cvShowImage("ldemlab",dm.lmap);

        int ix, iy;
        for (iy=min(lu.y,rb.y); iy<=max(lu.y,rb.y); iy++)
            for (ix=min(lu.x,rb.x); ix<=max(lu.x,rb.x); ix++)
                printf("%d, %d, %.3f,%.3f\n", ix, iy, dm.demg[iy*dm.wid+ix], dm.demhmin[iy*dm.wid+ix]);
        cvReleaseImage(&tmp);
    }
}

LONGLONG myGetFileSize(FILE *f)
{
    // set the file pointer to end of file
    fseeko(f, 0, SEEK_END);
    // get the file size
    LONGLONG retSize = ftello(f);
    // return the file pointer to the begin of file
    rewind(f);
    return retSize;
}

//主处理程序
void DoProcessing()
{

    LONGLONG fileSize = myGetFileSize(dfp);
    dFrmNum = fileSize / 180 / dsbytesiz;
	InitRmap (&rm);
	InitDmap (&dm);
	InitDmap (&gm);
	InitDmap (&ggm);
	onefrm= new ONEDSVFRAME[1];
	IplImage * col = cvCreateImage (cvSize (1024, rm.len*3),IPL_DEPTH_8U,3); 
	CvFont font;
	cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX, 1,1, 0, 2);

    int waitkeydelay=0;
	dFrmNo = 0;

    while (ReadOneDsvFrame ())  // 读取一帧数据（580个block），保存在onefrm中
	{
		if (dFrmNo%100==0)
			printf("%d (%d)\n",dFrmNo,dFrmNum);

		//每一帧的处理
		ProcessOneFrame ();

        //可视化
        char str[10];
        sprintf (str, "Fno%d", dFrmNo);
        cvPutText(dm.lmap, str, cvPoint(50,50), &font, CV_RGB(0,0,255));

        cvResize (rm.rMap, col);  // 距离图像 可视化
        cvShowImage("range image",col);
        cvResize (rm.lMap, col);  // 分割图像 可视化
        cvShowImage("region",col);
        cv::Mat zmapRGB;
        cv::applyColorMap(cvarrToMat(gm.zmap), zmapRGB, cv::COLORMAP_HOT);
        if (dm.lmap) cvShowImage("ldemlab",dm.lmap);    // 单帧 可行驶区域
//        if (dm.zmap) cv::imshow("zdem", zmapRGB);      // 高程图
        if (gm.lmap) cvShowImage("gdemlab",gm.lmap);    // 多帧 可行驶区域 凹凸障碍
        if (gm.smap) cvShowImage("gsublab",gm.smap);    // 属性图

        cv::setMouseCallback("gsublab", CallbackLocDem, 0);

		char WaitKey;
		WaitKey = cvWaitKey(waitkeydelay);
		if (WaitKey==27)
			break;
        if (WaitKey=='z') {     // 连续播放
			if (waitkeydelay==1)
				waitkeydelay=0;
			else
				waitkeydelay=1;
		}
        if (WaitKey == 'a') {     // Back
            dFrmNo -= 20;
            if (dFrmNo < 0) {
                dFrmNo = 0;
            }
            fseeko64(dfp, dFrmNo * dsbytesiz * BKNUM_PER_FRM, SEEK_SET);
            continue;
        }
        if (WaitKey == 'd') {     // Forword
            dFrmNo += 20;
            if (dFrmNo >= dFrmNum) {
                dFrmNo = dFrmNum - 1;
            }
            fseeko64(dfp, dFrmNo * dsbytesiz * BKNUM_PER_FRM, SEEK_SET);
            continue;
        }
		dFrmNo++;
	}

	ReleaseRmap (&rm);
	ReleaseDmap (&dm);
	ReleaseDmap (&gm);
	ReleaseDmap (&ggm);
	cvReleaseImage(&col);
	delete []onefrm;
}

int main (int argc, char *argv[])
{

	if (argc<3) {
        printf ("Usage : %s [infile] [calibfile]\n", argv[0]);
        printf ("[infile] DSV file.\n");
        printf ("[calibfile] define the calibration parameters of the DSV file.\n");
        printf ("[outfile] segmentation results to DSVL file.\n");
        printf ("[seglog] data association results to LOG file.\n");
        printf ("[videooutflg] 1: output video to default files, 0: no output.\n");
		exit(1);
	}

	if (!LoadCalibFile (argv[2])) {
        printf ("Invalid calibration file : %s.\n", argv[2]);
		getchar ();
		exit (1);
	}

    if ((dfp = fopen(argv[1], "r")) == NULL) {
		printf("File open failure : %s\n", argv[1]);
		getchar ();
		exit (1);
	}

	DoProcessing ();

    printf ("Labeling succeeded.\n");

    fclose(dfp);

    return 0;
}

