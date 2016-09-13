//-------------------------------------------------
//thetas-viewer.cpp
//
//since: 2016-07-29
//-------------------------------------------------

#include<cstdio>
#include <iostream>
#include <signal.h>
#include "opencv2/opencv.hpp"
#include <ssm.hpp>
#include "SSM-Image-HD720p.hpp"
#include "ThetaSConverter.hpp"

#define D_ANGLE_DEG 4

#define D_WIDTH_SIZE 16
#define MAX_WIDTH_SIZE 2000
#define MIN_WIDTH_SIZE 64

#define D_AOV_DEG 5
#define MAX_AOV_DEG 160
#define MIN_AOV_DEG 10

#define WINNAME_PERSPECTIVE "Perspective View"
#define WINNAME_EQUIRECTANGLE "Equirectangle View"
#define WINNAME_SOURCE "Source Image"

using namespace std;
using namespace cv;
using namespace sn;

bool gShutOff;

void ctrlC(int aStatus)
{
    signal(SIGINT, NULL);
    gShutOff = true;
}

// Ctrl-C による正常終了を設定
inline void setSigInt(){ signal(SIGINT, ctrlC); }

// 決め打ちのカメラ内部パラメータでTHETASの画像を透視射影に変換する
void makePerspectiveViewWithFixedParams( Mat img, Mat &dst, Matx33d prjMat, Size size);
void showHelp( const char* argv0 );

int main(int argc, char ** argv)
{
    //
    // オプション
    //
    
    int ssm_id = 0;
    bool show_equi = false;  // 正距円筒図法の表示有無
    bool show_src = false;    // ソース画像の表示有無
    int base_angle = 0;

    // オプション解析
    int opt;
    while( (opt = getopt(argc,argv,"i:esr:h")) != -1 ){
        
        switch(opt){
            case 'i':
                ssm_id = atoi(optarg);
                break;
            case 'e':
                show_equi = true;
                break;
            case 's':
                show_src = true;
                break;
            case 'r':
                base_angle = atoi(optarg);
                break;
            case 'h':
                showHelp( argv[0] );
                return 0;
        }
    }

	//
	// 初期パラメータ
	//
	ThetaSConverter conv;
    int theta = conv.GetPerspectiveDirection()[0];
    int phi = conv.GetPerspectiveDirection()[1];
    int aov = conv.GetPerspectiveAoV();
    int pers_width = conv.GetPerspectiveSize().width;
    bool blending = true;


	// SSM 初期化
	if(!initSSM()){
		cerr << "SSM Error : initSSM()" << endl;
		return 0;
	}

    SSMApi<ImageC3_HD720p> cam_image(SNAME_HD720P, ssm_id);

	if( !cam_image.open( ) ){
		cerr << "SSM Error : create()" << endl;
		return 1;
	}

	setSigInt();

    //==========================================================
    // ---> OPERATION
    //==========================================================
	//ループ処理の開始
	cerr << "Main Loop Started" << endl;
	Mat frm, equirect, perspect;
	while(!gShutOff){
		
		// 画像読み込み
		cam_image.readLast();
		ImageC3_HD720p_to_Mat(cam_image.data,&frm);

        // 画像表示
        if( show_src ){
            imshow( WINNAME_SOURCE, frm );
        }
        
        // 
        if( show_equi ){
            conv.Equirectangle(frm, equirect, blending);
            imshow( WINNAME_EQUIRECTANGLE, equirect );
        }
        
        conv.PerspectiveView(frm, perspect);
        imshow( WINNAME_PERSPECTIVE, perspect);
        
        
        //
        // キー操作
        //
        
        int key = waitKey(1);
        if( key == 'r') conv.SetBaseAngle(++base_angle); // ベース角度回転
        if( key == 'b') blending = !blending;            // 正距円筒図法のブレンディングON/OFF
        
        // 画像サイズ
        if( key == 'c' ){
            pers_width = MIN( MAX_WIDTH_SIZE, pers_width+D_WIDTH_SIZE);
            conv.SetPerspectiveSize( pers_width, pers_width*3/4 );
        }
        if( key == 'v' ){
            pers_width = MAX( MIN_WIDTH_SIZE, pers_width-D_WIDTH_SIZE);
            conv.SetPerspectiveSize( pers_width, pers_width*3/4 );
        }
        
        // リセット
        if( key == '0'){
            conv.SetPerspectiveDirection(theta=0.0, phi=0.0);  // 向き
            conv.SetPerspectiveAoV(aov=45.0); // 画角
        }
        
        // 画角
        if( key == 'z'){
            aov = MAX( MIN_AOV_DEG, aov-D_AOV_DEG);
            conv.SetPerspectiveAoV(aov);
        }
        if( key == 'x'){
            aov = MIN( MAX_AOV_DEG, aov+D_AOV_DEG);
            conv.SetPerspectiveAoV(aov);
        }
        
        // 正距円筒図のON/OFF
        if( key == 'e'){
            show_equi = !show_equi;
            if( !show_equi )
                destroyWindow(WINNAME_EQUIRECTANGLE);
        }
        
        // ソース画像のON/OFF
        if( key == 's'){
            show_src = !show_src;
            if( !show_src)
                destroyWindow(WINNAME_SOURCE);
        }
        
        // カメラ操作
        if( key == 65362 ){ // Up
            theta = MIN( 90, theta+D_ANGLE_DEG);
            conv.SetPerspectiveDirection(theta, phi);
        }
        if( key == 65364 ){ // Down
            theta = MAX( -90, theta-D_ANGLE_DEG);
            conv.SetPerspectiveDirection(theta, phi);
        }
        if( key == 65361 ){ // Left
            phi -= D_ANGLE_DEG;
            conv.SetPerspectiveDirection(theta, phi);
        }
        if( key == 65363 ){ // right
            phi += D_ANGLE_DEG;
            conv.SetPerspectiveDirection(theta, phi);
        }

        // key == Esc, 'q'
        if( key == 27 || key == 'q'){
			gShutOff = true;
			break;
		}


        usleep(25000);

	}
    // <--- OPERATION

    //==========================================================
    // ---> FINALIZE
    //==========================================================
	cam_image.close();

    endSSM();
    cerr << "End SSM" << endl;
    // <--- FINALIZE

    cout << "End Successfully" << endl;
    return 0;
}

// 決め打ちのカメラ内部パラメータでTHETASの画像を透視射影に変換する
void makePerspectiveViewWithFixedParams( Mat img, Mat &dst, Matx33d prjMat, Size size)
{
    // カメラ内部パラメータ
    double s = 181.0;
    cv::Matx33d K1(s, 0., 320, 0., s, 320., 0., 0., 1.);
    cv::Matx33d K2(s, 0., 958, 0., -s, 320., 0., 0., 1.);
    cv::Vec4d   D(0., 0., 0., 0.);
    Vec2d f1(K1(0,0), K1(1,1));
    Vec2d c1(K1(0,2), K1(1,2));
    Vec2d f2(K2(0,0), K2(1,1));
    Vec2d c2(K2(0,2), K2(1,2));
    
    // 透視射影画像から世界座標に変換する行列を求める
    Matx33d iR = prjMat.inv(DECOMP_SVD);
    
    // rempa用のマップ準備
    Mat map1(size, CV_16SC2), map2(size, CV_16SC1);
    
    // (i,j)に対する(u,v)の座標の計算
    for( int i=0; i<size.height; i++ )
    {
        float* m1f = map1.ptr<float>(i);
        float* m2f = map2.ptr<float>(i);
        short* m1 = (short*)m1f;
        ushort *m2 = (ushort*)m2f;
        
        double _x = i*iR(0,1) + iR(0,2);
        double _y = i*iR(1,1) + iR(1,2);
        double _w = i*iR(2,1) + iR(2,2);
        
        for( int j=0; j<size.width; j++ )
        {
            double x = _x/_w, y = _y/_w;
            double r = sqrt(x*x + y*y);
            double theta = atan(r);
            
            // 歪み計算
            //double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta4*theta4;
            //double theta_d = theta * (1 + D[0]*theta2 + D[1]*theta4 + D[2]*theta6 + D[3]*theta8);
            double theta_d = theta;
            
            double scale = (r == 0) ? 1.0 : theta_d / r;
            double u, v;
            if(_w>0.0){
                u = f1[0]*x*scale + c1[0];
                v = f1[1]*y*scale + c1[1];
            }
            else{
                u = f2[0]*x*scale + c2[0];
                v = f2[1]*y*scale + c2[1];
            }
            
            // map 作成
            int iu = cv::saturate_cast<int>(u*cv::INTER_TAB_SIZE);
            int iv = cv::saturate_cast<int>(v*cv::INTER_TAB_SIZE);
            m1[j*2+0] = (short)(iu >> cv::INTER_BITS);
            m1[j*2+1] = (short)(iv >> cv::INTER_BITS);
            m2[j] = (ushort)((iv & (cv::INTER_TAB_SIZE-1))*cv::INTER_TAB_SIZE + (iu & (cv::INTER_TAB_SIZE-1)));
            
            
            // 次の j のための準備
            _x += iR(0,0);
            _y += iR(1,0);
            _w += iR(2,0);
        }
        
    }
    
    // 変換実行
    remap(img, dst, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
    
}


void showHelp( const char* argv0 )
{
    // arg[0]から実行ファイル名を取り出す
    string exe_name = argv0;
    size_t slash_pos = exe_name.find_last_of( '/' );
    exe_name.erase(0, slash_pos+1);
    
    // 書式
    fprintf( stdout, "\n\n" );
    fprintf( stdout, "\033[1m書式\033[0m\n");
    fprintf( stdout, "\t\033[1m%s\033[0m [-i cameraID] [-e] [-s] [-r rot]\n", exe_name.c_str() );
    fprintf( stdout, "\t\033[1m%s\033[0m [-h]\n", exe_name.c_str() );
    fprintf( stdout, "\n" );
    
    // 説明
    fprintf( stdout, "\n" );
    fprintf( stdout, "\033[1m説明\033[0m\n" );
    fprintf( stdout, "\t\033[1m-i\033[0m\tSSMのIDを指定する\n" );
    fprintf( stdout, "\t\033[1m-e\033[0m\t起動時に正距円筒図画像を表示する\n" );
    fprintf( stdout, "\t\033[1m-s\033[0m\t起動時にソース画像を表示する\n" );
    fprintf( stdout, "\t\033[1m-r\033[0m\t表示画像の初期回転角(90度毎)を指定する[0-3]\n" );
    fprintf( stdout, "\t\033[1m-h\033[0m\tこのヘルプを表示する\n" );
    fprintf( stdout, "\n" );
    
    // 使用方法
    fprintf( stdout, "\n\033[1m使用方法\033[0m\n" );
    fprintf( stdout, "\t%-18s視点の仰角と方位角の向きを%d度ずつ変更する\n", "矢印キー", D_ANGLE_DEG );
    fprintf( stdout, "\t%-16s基準角度を90度回転させる\n", "R キー" );
    fprintf( stdout, "\t%-16s画角を%d度狭める(ズームイン)\n", "Z キー", D_AOV_DEG );
    fprintf( stdout, "\t%-16s画角を%d度広げる(ズームアウト)\n", "X キー", D_AOV_DEG );
    fprintf( stdout, "\t%-16s画像の幅を%d[pix]広げる(拡大)\n", "C キー", D_WIDTH_SIZE );
    fprintf( stdout, "\t%-16s画角の幅を%d[pix]狭める(縮小)\n", "V キー", D_WIDTH_SIZE );
    fprintf( stdout, "\t%-16s視点の方向と画角をリセットする\n", "0 キー" );
    fprintf( stdout, "\t%-16s正距円筒図の表示／非表示\n", "E キー" );
    fprintf( stdout, "\t%-16sソース画像の表示／非表示\n", "S キー" );
    
    fprintf( stdout, "\n" );
    
    // フッター
    fprintf( stdout, "\t\t\t\t\t\t\t2016年09月\n" );
    fprintf( stdout, "\n\n" );

}
