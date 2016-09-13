#ifndef __THETA_S_CONVERTER_H__
#define __THETA_S_CONVERTER_H__

#include <opencv2/opencv.hpp>

using namespace cv;

namespace sn{

    class ThetaSConverter{
        
    private:
        // カメラパラメータ
        int _base_angle; // カメラ傾きアングル(0,1,2,3)
        double _focus; // 立体射影の焦点位置
        double _radius; // 仰角0度に対する位置の半径[pix]
        Vec2d _center_f; // 正面センサの中心座標[pix]
        Vec2d _center_b; // 後面センサの中心座標[pix]
        double _dangle; // 後面センサの相対傾き角度[rad]
        
        // 正距円筒図法関連パラメータ
        Size _equirect_size; // 正距円筒図のサイズ[pix,pix]
        int _equirect_shift; // 正面画像左端の位置
        Mat _eqmap1, _eqmap2;  // 正距円筒図の変換マップ(ブレンドなし)
        int _blending_margin; // 正距円筒図でブレンディングする時のマージン幅[pix]
        Mat _eqmapbf1, _eqmapbf2; // 正距円筒図のブレンド部分の変換マップ（後面->正面用)
        Mat _eqmapfb1, _eqmapfb2; // 正距円筒図のブレンド部分の変換マップ（正面->後面用)
        
        // 透視射影関連パラメータ
        Size _perspective_size; // 透視射影画像サイズ
        double _aov;   // 画角[deg]
        double _theta; // 仰角[deg]
        double _phi;   // 方位角[deg]
        
    public:
        // コンストラクタ
        ThetaSConverter(void);
        
        // カメラの基準角度を90度ごと設定する(0,1,2,3)
        void SetBaseAngle( int angle );
        int GetBaseAngle(void){ return _base_angle; }
        
        // 正距円筒図のサイズを設定する
        void SetEquirectSize( int w, int h );
        void SetEquirectSize( Size size ){ this->SetEquirectSize(size.width, size.height); }
        Size GetEquirectSize( void ){ return _equirect_size; }
        
        // 正距円筒図を作成する
        void Equirectangle( Mat &src, Mat &dst, bool blending=true, int interpolation=CV_INTER_LINEAR );
        
        // 透視画像を取得する
        void PerspectiveView( Mat &src, Mat &dst );
        
        // 透視画像のサイズ
        void SetPerspectiveSize( int w, int h);
        void SetPerspectiveSize( Size size ){ this->SetPerspectiveSize( size.width, size.height); }
        Size GetPerspectiveSize( void ){ return _perspective_size; }
        
        // 透視画像の向き
        void SetPerspectiveDirection( double theta_deg, double phi_deg );
        Vec2d GetPerspectiveDirection( void ){ return Vec2d(_theta, _phi); }
        
        // 透視画像の画角
        void SetPerspectiveAoV( double aov_deg ){ _aov = aov_deg; }
        double GetPerspectiveAoV( void ){ return _aov; }
        
    private:
        // 正距円筒図のための変換マップを作成する
        void makeEquirectangleMap( Mat &map1, Mat &map2 );
        void makeEquirectangleMarginMap( Mat &mapf1, Mat &mapf2, Mat &mapb1, Mat &mapb2 );
    };
}

#endif // __THETA_S_CONVERTER_H__
