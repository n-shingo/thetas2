#include "ThetaSConverter.hpp"

using namespace std;
using namespace cv;
using namespace sn;

// コンストラクタ
ThetaSConverter::ThetaSConverter(void){

	// カメラパラメータ初期値(SN:170159向け)
    _base_angle = 0;
	_focus = 2.0;
	_radius = 284.0;
	_center_f = Vec2d(319.0, 319.0);
    _center_b = Vec2d(959.0, 319.0);
	_dangle = -0.004;

	// 正距円筒図パラメータ
	_equirect_size = Size(960, 480);
    _equirect_shift = _equirect_size.width/4; // 正面画像左端の位置
	_blending_margin = 10;
    makeEquirectangleMap( _eqmap1, _eqmap2 );
    makeEquirectangleMarginMap(_eqmapbf1, _eqmapbf2, _eqmapfb1, _eqmapfb2);
    
    // 透視射影関連パラメータ
    _perspective_size = Size(640, 480); // 透視射影画像サイズ
    _aov = 45.0; // 画角[deg]
    _theta = 0.0;  // 仰角[deg]
    _phi = 0.0;    // 方位角[deg]

}

// 正距円筒図のサイズを設定する[pix]
void ThetaSConverter::SetEquirectSize(int w, int h ){

	assert( w > 0 && h > 0 );
	
	_equirect_size.width = w;
	_equirect_size.height = h;
    _equirect_shift = w/4;
    makeEquirectangleMap( _eqmap1, _eqmap2 );
    makeEquirectangleMarginMap(_eqmapbf1, _eqmapbf2, _eqmapfb1, _eqmapfb2);
}


// カメラの基準角度を90度ごと設定する(0,1,2,3)
void ThetaSConverter::SetBaseAngle( int angle )
{
    assert( angle >= 0 );
    _base_angle = angle%4;
    makeEquirectangleMap( _eqmap1, _eqmap2 );
    makeEquirectangleMarginMap(_eqmapbf1, _eqmapbf2, _eqmapfb1, _eqmapfb2);
}


// Dual Fishey 画像から正距円筒図を取得する
void ThetaSConverter::Equirectangle( Mat &src, Mat &dst, bool blending, int interpolation ){
	
    assert( src.type() == CV_8UC3 );
    
    // 諸定数
    const int w = _equirect_size.width;
    const int h = _equirect_size.height;
    const int margin = _blending_margin;
    const int shift = _equirect_shift;

    // ブレンディングなし画像作成
    dst = Mat( _equirect_size, src.type() );
    remap( src, dst, _eqmap1, _eqmap2, interpolation );
    if( !blending ) return;
    
    // ブレンディング部画像作成
    Mat bf_img( 2*margin, h, src.type());
    Mat fb_img( 2*margin, h, src.type());
    remap( src, bf_img, _eqmapbf1, _eqmapbf2, interpolation);
    remap( src, fb_img, _eqmapfb1, _eqmapfb2, interpolation);

    // ブレンディングする
    for( int j=0; j<h; j++ ){
        
        uchar* dptr = dst.data + j*dst.step;
        uchar* bfptr = bf_img.data + j*bf_img.step;
        uchar* fbptr = fb_img.data + j*fb_img.step;
        
        for( int i=-margin; i<margin; i++ )
        {
            // ブレンド率
            double a = (fabs(i)+margin+1)/(2.0*margin+1);
            
            // 後面から前面
            int x = (shift + i + w)%w;
            dptr[3*x+0] = (uchar)cvRound(a*dptr[3*x+0] + (1.0-a)*bfptr[3*(i+margin)+0]);
            dptr[3*x+1] = (uchar)cvRound(a*dptr[3*x+1] + (1.0-a)*bfptr[3*(i+margin)+1]);
            dptr[3*x+2] = (uchar)cvRound(a*dptr[3*x+2] + (1.0-a)*bfptr[3*(i+margin)+2]);
            
            // 前面から後面
            x = (x+w/2)%w;
            dptr[3*x+0] = (uchar)cvRound(a*dptr[3*x+0] + (1.0-a)*fbptr[3*(i+margin)+0]);
            dptr[3*x+1] = (uchar)cvRound(a*dptr[3*x+1] + (1.0-a)*fbptr[3*(i+margin)+1]);
            dptr[3*x+2] = (uchar)cvRound(a*dptr[3*x+2] + (1.0-a)*fbptr[3*(i+margin)+2]);
            
        }
    }
}

// 透視画像のサイズを設定する
void ThetaSConverter::SetPerspectiveSize( int w, int h ){
    _perspective_size = Size(w,h);
}


// 透視画像の向きを設定する
void ThetaSConverter::SetPerspectiveDirection( double theta_deg, double phi_deg )
{
    _theta = theta_deg;
    _phi = phi_deg;
}


// 透視画像を取得する
void ThetaSConverter::PerspectiveView( Mat &src, Mat &dst )
{
    // 諸定数
    const int w = _perspective_size.width;
    const int h = _perspective_size.height;
    const double angle = _base_angle * CV_PI/2.0;
    const double tanw2 = tan(_aov*CV_PI/180.0/2.0);  // tan(aov/w)

    // 回転行列
    const double the_rad = _theta*CV_PI/180.0;
    const double ct = cos(the_rad);
    const double st = sin(the_rad);
    const double phi_rad = _phi*CV_PI/180.0;
    const double cp = cos(phi_rad);
    const double sp = sin(phi_rad);
    const double R[3][3] = { {cp, ct*sp, -st*sp}, {-sp, ct*cp, -st*cp}, {0, st, ct} };
    
    // 変換マップ作成
    Mat mapu( _perspective_size, CV_32F );
    Mat mapv( _perspective_size, CV_32F );

    for( int j=0; j<h; j++ ){
        
        float* pu = (float*)(mapu.data + j*mapu.step);
        float* pv = (float*)(mapv.data + j*mapv.step);

        double t = (j-h/2)*2*tanw2/w;
        for( int i=0; i<w; i++ ){
            double s = (i-w/2)*2*tanw2/w;
            
            double r1 = sqrt( s*s + 1.0 + t*t);
            double x2 = s/r1;
            double y2 = 1.0/r1;
            double z2 = -t/r1;
            
            double x3 = R[0][0]*x2 + R[0][1] * y2 + R[0][2]*z2;
            double y3 = R[1][0]*x2 + R[1][1] * y2 + R[1][2]*z2;
            double z3 =              R[2][1] * y2 + R[2][2]*z2;
            
            double x4, z4, u, v;
            if( y3 >= 0.0 )  // 正面
            {
                x4 = _focus * x3 / (y3+_focus);
                z4 = _focus * z3 / (y3+_focus);

                double _x4 = cos(angle) * x4 + sin(angle) * z4;
                double _z4 = -sin(angle) * x4 + cos(angle) * z4;

                u = -_z4 * _radius + _center_f[0];
                v = -_x4 * _radius + _center_f[1];
            }
            else  // 後面
            {
                x4 = -_focus* x3 / (y3-_focus);
                z4 = -_focus* z3 / (y3-_focus);
                double _x4 = cos(angle+_dangle) * x4 + sin(angle+_dangle) * z4;
                double _z4 = -sin(angle+_dangle) * x4 + cos(angle+_dangle) * z4;
                
                u = _z4 * _radius + _center_b[0];
                v = -_x4 * _radius + _center_b[1];
            }
            
            pu[i] = (float)u;
            pv[i] = (float)v;
        }
    }
    
    // 変換
    dst = Mat( _perspective_size, src.type());
    remap(src, dst, mapu, mapv, CV_INTER_LINEAR);
}



// Dual Fish Eye 画像から正距円筒図に変換するためのmapを作成する
void ThetaSConverter::makeEquirectangleMap( Mat &map1, Mat &map2 )
{
	const int w = _equirect_size.width;
	const int h = _equirect_size.height;
	const int shift = _equirect_shift;
    const double angle = _base_angle * CV_PI/2.0;


	Mat mapu( _equirect_size, CV_32F );
	Mat mapv( _equirect_size, CV_32F );

	for( int j=0; j<h; j++ ){
		double theta = CV_PI * j / h;
		double cos_the = cos(theta);
		double sin_the = sin(theta);

		float* pu = (float*)(mapu.data + j*mapu.step);
		float* pv = (float*)(mapv.data + j*mapv.step);
		
		// 正面
		for( int i=0; i<w/2; i++ ){
			double phi = 2.0*CV_PI * i / w;
			double cos_phi = cosf(phi);
			double sin_phi = sinf(phi);

			double _x = _focus * (cos_phi * sin_the ) / (sin_phi * sin_the + _focus);
			double _z = _focus * cos_the / (sin_phi * sin_the + _focus );
            
            double _x2 = cos(angle) * _x - sin(angle) * _z;
            double _z2 = sin(angle) * _x + cos(angle) * _z;

			double u = -_z2 * _radius + _center_f[0];
			double v = _x2 * _radius + _center_f[1];

			pu[(shift+i)%w] = (float)u;
			pv[(shift+i)%w] = (float)v;
		}

		// 後面
		for( int i=w/2; i<w; i++ ){
			double phi = 2.0 * CV_PI * i / w;
			double cos_phi = cosf(phi);
			double sin_phi = sinf(phi);

			double _x = -_focus * (cos_phi * sin_the) / (sin_phi*sin_the - _focus);
			double _z = -_focus * cos_the / (sin_phi * sin_the - _focus );

            double _x2 = cos(angle+_dangle) * _x - sin(angle+_dangle) * _z;
            double _z2 = sin(angle+_dangle) * _x + cos(angle+_dangle) * _z;

			double u = _z2 * _radius + _center_b[0];
			double v = _x2 * _radius + _center_b[1];

			pu[(shift+i)%w] = (float)u;
			pv[(shift+i)%w] = (float)v;

		}

	}

	// 高速化のため整数マップに変換
	map1 = Mat( _equirect_size, CV_16SC2);
	map2 = Mat( _equirect_size, CV_16SC1);
	convertMaps( mapu, mapv, map1, map2, CV_16SC2 );
}


// Dual Fish Eye 画像から正距円筒図に変換するためのマージン部分のmapを作成する
void ThetaSConverter::makeEquirectangleMarginMap( Mat &mapbf1, Mat &mapbf2, Mat &mapfb1, Mat&mapfb2 ){
    
    const int w = _equirect_size.width;
    const int h = _equirect_size.height;
    const int margin = _blending_margin;
    const double angle = _base_angle * CV_PI/2.0;
    
    Mat mapu( Size(2*margin, h), CV_32F );
    Mat mapv( Size(2*margin, h), CV_32F );
    
    // 後面から前面用マップ作成
    for( int j=0; j<h; j++ ){
        double theta = CV_PI * j / h;
        double cos_the = cosf(theta);
        double sin_the = sinf(theta);
        
        float* pu = (float*)(mapu.data + j*mapu.step);
        float* pv = (float*)(mapv.data + j*mapv.step);
        
        // 正面
        for( int i=-margin; i<0; i++ ){
            double phi = 2.0*CV_PI * i / w;
            double cos_phi = cosf(phi);
            double sin_phi = sinf(phi);
            
            double _x = _focus * (cos_phi * sin_the ) / (sin_phi * sin_the + _focus);
            double _z = _focus * cos_the / (sin_phi * sin_the + _focus );
            
            double _x2 = cos(angle) * _x - sin(angle) * _z;
            double _z2 = sin(angle) * _x + cos(angle) * _z;

            double u = -_z2 * _radius + _center_f[0];
            double v = _x2 * _radius + _center_f[1];
            
            pu[margin+i] = (float)u;
            pv[margin+i] = (float)v;
        }
        
        // 後面
        for( int i=0; i<margin; i++ ){
            double phi = 2.0 * CV_PI * i / w;
            double cos_phi = cosf(phi);
            double sin_phi = sinf(phi);
            
            double _x = -_focus * (cos_phi * sin_the) / (sin_phi*sin_the - _focus);
            double _z = -_focus * cos_the / (sin_phi * sin_the - _focus );
            
            double _x2 = cos(angle+_dangle) * _x - sin(angle+_dangle) * _z;
            double _z2 = sin(angle+_dangle) * _x + cos(angle+_dangle) * _z;
            
            double u = _z2 * _radius + _center_b[0];
            double v = _x2 * _radius + _center_b[1];
            
            pu[margin+i] = (float)u;
            pv[margin+i] = (float)v;
            
        }
        
    }
    
    // 高速化のため整数マップに変換
    mapbf1 = Mat( mapu.size(), CV_16SC2);
    mapbf2 = Mat( mapv.size(), CV_16SC1);
    convertMaps( mapu, mapv, mapbf1, mapbf2, CV_16SC2 );
    
    
    // 前面から後面用マップ作成
    for( int j=0; j<h; j++ ){
        double theta = CV_PI * j / h;
        double cos_the = cosf(theta);
        double sin_the = sinf(theta);
        
        float* pu = (float*)(mapu.data + j*mapu.step);
        float* pv = (float*)(mapv.data + j*mapv.step);
        
        // 正面
        for( int i=w/2; i<w/2+margin; i++ ){
            double phi = 2.0*CV_PI * i / w;
            double cos_phi = cosf(phi);
            double sin_phi = sinf(phi);
            
            double _x = _focus * (cos_phi * sin_the ) / (sin_phi * sin_the + _focus);
            double _z = _focus * cos_the / (sin_phi * sin_the + _focus );
            
            double _x2 = cos(angle) * _x - sin(angle) * _z;
            double _z2 = sin(angle) * _x + cos(angle) * _z;

            double u = -_z2 * _radius + _center_f[0];
            double v = _x2 * _radius + _center_f[1];
            
            pu[margin+i-w/2] = (float)u;
            pv[margin+i-w/2] = (float)v;
        }
        
        // 後面
        for( int i=w/2-margin; i<w/2; i++ ){
            double phi = 2.0 * CV_PI * i / w;
            double cos_phi = cosf(phi);
            double sin_phi = sinf(phi);
            
            double _x = -_focus * (cos_phi * sin_the) / (sin_phi*sin_the - _focus);
            double _z = -_focus * cos_the / (sin_phi * sin_the - _focus );
            
            double _x2 = cos(angle+_dangle) * _x - sin(angle+_dangle) * _z;
            double _z2 = sin(angle+_dangle) * _x + cos(angle+_dangle) * _z;
            
            double u = _z2 * _radius + _center_b[0];
            double v = _x2 * _radius + _center_b[1];
            
            pu[margin+i-w/2] = (float)u;
            pv[margin+i-w/2] = (float)v;
            
        }
        
    }
    
    // 高速化のため整数マップに変換
    mapfb1 = Mat( mapu.size(), CV_16SC2);
    mapfb2 = Mat( mapv.size(), CV_16SC1);
    convertMaps( mapu, mapv, mapfb1, mapfb2, CV_16SC2 );

}
