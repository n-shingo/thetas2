//
//  main.cpp
//  ThetaS2
//
//  Created by shingo on 2016/07/29.
//  Copyright (c) 2016年 shingo. All rights reserved.
//

#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
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

void showHelp( const char* argv0 );

int main(int argc, char * argv[]) {

    //
    // オプション
    //
    
    int cam_id = 0;
    bool equi_view = false;  // 正距円筒図法の表示有無
    bool src_view = false;    // ソース画像の表示有無
    int base_angle = 0;

    int opt;
    while( (opt = getopt(argc,argv,"i:esr:h")) != -1 ){
        
        switch(opt){
            case 'i':
                cam_id = atoi(optarg);
                break;
            case 'e':
                equi_view = true;
                break;
            case 's':
                src_view = true;
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
    // 初期値
    //
    VideoCapture vc(cam_id);
    ThetaSConverter conv;
    int theta = conv.GetPerspectiveDirection()[0];
    int phi = conv.GetPerspectiveDirection()[1];
    int aov = conv.GetPerspectiveAoV();
    int pers_width = conv.GetPerspectiveSize().width;
    bool blending = true;
    
    //
    // ループ処理開始
    //
    Mat frm, equirect, perspect;
    while(1)
    {
        vc >> frm;
        
        //
        // 画像表示
        //
        if( src_view ){
            imshow( WINNAME_SOURCE, frm );
        }
        
        if( equi_view ){
            conv.Equirectangle(frm, equirect, blending);
            imshow( WINNAME_EQUIRECTANGLE, equirect );
        }
        
        conv.PerspectiveView(frm, perspect);
        imshow( WINNAME_PERSPECTIVE, perspect);
        
        
        //
        // キー操作
        //
        
        int key = waitKey(1);
        if( key == 27 ) break;                           // ESC 終了
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
            equi_view = !equi_view;
            if( !equi_view )
                destroyWindow(WINNAME_EQUIRECTANGLE);
        }
        
        // ソース画像のON/OFF
        if( key == 's'){
            src_view = !src_view;
            if( !src_view)
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
        
    }
    
    return 0;
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
    fprintf( stdout, "\t\033[1m-i\033[0m\tThetaSのカメラIDを指定する\n" );
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
