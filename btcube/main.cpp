/*! 
  @file bt_simple.cpp
	
  @brief Bulletによる物理シミュレーション
		 シンプルな剛体落下
 
  @author Makoto Fujisawa
  @date 2013-03
*/

#pragma comment (lib, "glew32.lib")
//progma はライブラリをリンカにリンクしている

#pragma comment (lib, "LinearMath.lib")
#pragma comment (lib, "BulletCollision.lib")
#pragma comment (lib, "BulletDynamics.lib")
#pragma comment (lib, "BulletSoftBody.lib")

//softbody
#pragma comment (lib, "BulletSoftBody.lib")

//-----------------------------------------------------------------------------
// インクルードファイル
//-----------------------------------------------------------------------------
#include "utils.h"


#include "rx_model_sa.h"
//#include "rx_mesh_sa.h"
//#include "rx_obj_sa.h"


//softbody
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBody.h>

#include "rx_model.h"

#ifdef _DEBUG
 #pragma comment (lib, "rx_modeld.lib")
#else
 #pragma comment (lib, "rx_model.lib")
#endif

#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"



//-----------------------------------------------------------------------------
// MARK:定義/定数
//-----------------------------------------------------------------------------
const GLfloat RX_LIGHT0_POS[4] = {  2.0f, 4.0f, 1.0f, 0.0f };
const GLfloat RX_LIGHT1_POS[4] = { -1.0f, -10.0f, -1.0f, 0.0f };

const GLfloat RX_LIGHT_DIFF[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat RX_LIGHT_SPEC[4] = { 0.7f, 0.6f, 0.6f, 1.0f };
const GLfloat RX_LIGHT_AMBI[4] = { 0.3f, 0.3f, 0.3f, 1.0f };

const GLfloat RX_FOV = 45.0f;

const double DT = 0.01;					//!< 時間ステップ幅Δt

//-----------------------------------------------------------------------------
// MARK:グローバル変数
//-----------------------------------------------------------------------------
//! ウィンドウサイズ
int g_iWinW = 720;
int g_iWinH = 720;
int g_iMouseButton = -1;	//!< マウスボタンの状態
bool g_bIdle;				//!< アニメーションフラグ

//! トラックボール(マウスによる視点回転用)
rxTrackball g_tbView;

// シャドウマッピング(影付け用)
rxShadowMap g_ShadowMap;
int g_iShadowMapSize = 512;

// Bullet
btDynamicsWorld* g_pDynamicsWorld;	//!< Bulletワールド
btAlignedObjectArray<btCollisionShape*>	g_vCollisionShapes;		//!< 剛体オブジェクトの形状を格納する動的配列

// マウスピック
btVector3 g_vPickPos;
btRigidBody *g_pPickBody = 0;
btPoint2PointConstraint *g_pPickConstraint = 0;
btSoftBody::Node *g_pPickNode = 0;
double g_fPickDist = 0.0;

//追加 練習問題２
btAlignedObjectArray<btRigidBody*> g_pManyBody;
//追加　練習問題３
const btScalar CUBE_HALF_EXTENTS = 0.2;    // 立方体の変の長さの半分(中心から辺までの距離)
const btScalar GROUND_HEIGHT = 0.0;
btTransform trans;    // 剛体オブジェクトの位置姿勢を格納する変数(行列)
btQuaternion qrot(0, 0, 0, 1);


//追加　練習問題４ ex4
#define LEFT 0x4D /* → */
#define RIGHT 0x4B /* ← */
btRigidBody* g_MoveBody;//*をつけるべきなのかわからない

btScalar restitution = 0.8;     //反発

//追加　練習問題8　ex8
double eye_pos[3], eye_dir[3];
double init_pos[3] = {0, 0, 0};
double init_dir[3] = {0, 0, -1};

//追加　練習問題9 ex9
enum CollisionGroup{
    RX_COL_NOTHING = 0, // 0000
    RX_COL_GROUND = 1,
    RX_COL_GROUP1 = 2,
    RX_COL_GROUP2 = 4 //0100
};

//mikio
btRigidBody* g_MainBody;//*をつけるべきなのかわからない
int Onsomething = 0;
int framerate = 0;

int rightward = -1;
int leftward = -1;
int forwardd = -1;
int backwardd = -1;

int right_cnt = 0;
int left_cnt = 0;
int forward_cnt = 0;
int backward_cnt = 0;

btRigidBody* scaffold_1; //*をつけないとno matching constructerと表示されるのはなぜ？？
btRigidBody* scaffold_2;
btRigidBody* scaffold_3;//slider stoper
btRigidBody* scaffold_4;//slider
btRigidBody* scaffold_5;//slider stoper
btRigidBody* scaffold_6;
btRigidBody* scaffold_7;
btRigidBody* scaffold_8;



int jumping = -1;
int jump_cnt = 0;

int body_track_eye = -1;

btSliderConstraint *sliderr;
int t;
btScalar sliderHeight = 3.3;
int accel = -1;
int accel_cnt = 0;

int clear = -1;
btRigidBody* owan;
btRigidBody* board;
btRigidBody* knife;

int started = -1;

btRigidBody* goal_rigid;
btVector3 owan_p;

btVector3 board_p;

static void DrawString(string str, int w, int h, int x0, int y0)
{
    glDisable(GL_LIGHTING);
    // 平行投影にする
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, w, h, 0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    // 画面上にテキスト描画
    glRasterPos2f(x0, y0);
    int size = (int)str.size();
    for(int i = 0; i < size; ++i){
        char ic = str[i];
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, ic);
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

//-----------------------------------------------------------------------------
// MARK:Bullet用関数
//-----------------------------------------------------------------------------
/*!
 * Bullet剛体(btRigidBody)の作成
 * @param[in] mass 質量
 * @param[in] init_tras 初期位置・姿勢
 * @param[in] shape 形状
 * @param[in] index オブジェクト固有の番号
 * @return 作成したbtRigidBody
 */
btRigidBody* CreateRigidBody(double mass, const btTransform& init_trans, btCollisionShape* shape, int index, int GROUPme, int GROUPyou)
{
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	// 質量が0ならば静的な(static)オブジェクトとして設定，
	bool isDynamic = (mass != 0.0);

	// 形状から慣性テンソルを計算
	btVector3 inertia(0, 0, 0);
	if(isDynamic)
		shape->calculateLocalInertia(mass, inertia);
    //inertiaは参照渡し。。？

	// 初期位置，姿勢の設定
	btDefaultMotionState* motion_state = new btDefaultMotionState(init_trans);

	// 質量，慣性テンソル(回転しやすさ)，形状，位置・姿勢情報を一つの変数にまとめる
	btRigidBody::btRigidBodyConstructionInfo rb_info(mass, motion_state, shape, inertia);

	// 剛体オブジェクト(btRigidBody)の生成
	btRigidBody* body = new btRigidBody(rb_info);

	// 剛体オブジェクトに番号を設定
	body->setUserIndex(index);

	// Bulletワールドに剛体オブジェクトを追加
	g_pDynamicsWorld->addRigidBody(body, GROUPme, GROUPyou);

	return body;
}


/*!
 * 剛体オブジェクトの追加
 */
void SetRigidBodies(void)
{
	btTransform trans;	// 剛体オブジェクトの位置姿勢を格納する変数(行列)
	trans.setIdentity();// 位置姿勢行列の初期化

	const btScalar CUBE_HALF_EXTENTS = 0.2;	// 立方体の変の長さの半分(中心から辺までの距離)
	const btScalar GROUND_HEIGHT = 0.0;		// 地面の高さ

	
	// ----- 地面(質量0のx-z平面上で平べったい直方体で表現)の追加 -----
	btCollisionShape *ground_shape = new btBoxShape(btVector3(20, CUBE_HALF_EXTENTS, 20));	// 形状
	trans.setOrigin(btVector3(0, GROUND_HEIGHT-CUBE_HALF_EXTENTS, 0));	// 上の面がy=0になるように設定

	// 剛体オブジェクト(Static)生成
	btRigidBody* body0 = CreateRigidBody(0.0, trans, ground_shape, 0, RX_COL_GROUND, RX_COL_GROUP1 | RX_COL_GROUP2);	// Body
    
    body0->setFriction(0);
	
	g_vCollisionShapes.push_back(ground_shape); // 最後に破棄(delete)するために形状データを格納しておく
	// ----- ここまで (地面の追加) -----

	//wall　壁 mikio
    {
        btCollisionShape *ground_shape = new btBoxShape(btVector3(20, CUBE_HALF_EXTENTS, 20));    // 形状
        trans.setOrigin(btVector3(0, GROUND_HEIGHT-CUBE_HALF_EXTENTS, 0));    // 上の面がy=0になるように設定

        // 剛体オブジェクト(Static)生成
        btRigidBody* body0 = CreateRigidBody(0.0, trans, ground_shape, 0, RX_COL_GROUND, RX_COL_GROUP1 | RX_COL_GROUP2);    // Body
        
        body0->setFriction(0);
        
        g_vCollisionShapes.push_back(ground_shape);
    }
    
    
	// ----- 立方体オブジェクト追加 -----
	// 形状設定
	btCollisionShape *box_shape = new btBoxShape(btVector3(CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS));

	// 初期位置・姿勢
	btQuaternion qrot(0, 0, 0, 1);
	trans.setIdentity();// 位置姿勢行列の初期化
	trans.setOrigin(btVector3(0, GROUND_HEIGHT+10.0*CUBE_HALF_EXTENTS, 0));
	trans.setRotation(qrot);	// 四元数を行列に変換して姿勢行列に掛け合わせる

	// 剛体オブジェクト生成
	//btRigidBody* body1 = CreateRigidBody(1.0, trans, box_shape, 0,RX_COL_GROUP1, RX_COL_GROUND);
	
	//g_vCollisionShapes.push_back(box_shape); // 最後に破棄(delete)するために形状データを格納しておく
	// ----- ここまで (立方体オブジェクト追加) -----


	// すり抜け防止用Swept sphereの設定(CCD:Continuous Collision Detection)
//	body1->setCcdMotionThreshold(CUBE_HALF_EXTENTS);
//	body1->setCcdSweptSphereRadius(0.05*CUBE_HALF_EXTENTS);
}

//mikio 追加 視点
void QuaternionToEuler(const btQuaternion &q, btVector3 &euler)
{
  btScalar w = q.getW();
  btScalar x = q.getX();
  btScalar y = q.getY();
  btScalar z = q.getZ();
  euler[0] = asin(-2.0*(x*z-w*y));                    // yaw
  euler[1] = atan2(2.0*(y*z+w*x), (w*w-x*x-y*y+z*z));    // pitch
  euler[2] = atan2(2.0*(x*y+w*z), (w*w+x*x-y*y-z*z));    // roll
}

void MoveCameraWithRigid(rxTrackball &view, btRigidBody *body)
{
  btVector3 r = btVector3(0, 2, 7);

  // 剛体の位置と姿勢(四元数)
  btTransform trans;
  body->getMotionState()->getWorldTransform(trans);
  btVector3 p = trans.getOrigin();
  btQuaternion q = trans.getRotation();

  // 視点に対して横方向の回転(yaw)だけを残す
  btVector3 e;
  QuaternionToEuler(q, e);
  q.setEulerZYX(0.0, e[0], 0.0);
  
  q = q.inverse();

  // 視点変更用のトラックボールの回転中心が常に原点にあるので，
  // 先に回転させてから，平行移動を行う

  // 視線方向を回転
  double qd[4];
  qd[0] = q[3]; qd[1] = q[0]; qd[2] = q[1]; qd[3] = q[2];
  view.SetQuaternion(qd);

  // 剛体の後ろに視点を移動
  btVector3 epos = quatRotate(q, p)+r;
  view.SetTranslation(-epos[0], -epos[1]);
  view.SetScaling(-epos[2]);
}



/*!
 * Bullet初期化
 */
void InitBullet(void)
{
	// 衝突検出方法の選択(デフォルトを選択)
	btDefaultCollisionConfiguration *config = new btDefaultCollisionConfiguration();
	btCollisionDispatcher *dispatcher = new btCollisionDispatcher(config);
    
    //追加mikio 3dモデル
    btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

	// ブロードフェーズ法の設定(Dynamic AABB tree method)
	btDbvtBroadphase *broadphase = new btDbvtBroadphase();

	// 拘束(剛体間リンク)のソルバ設定
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	// Bulletのワールド作成
	g_pDynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config);

	// 重力加速度の設定(OpenGLに合わせてy軸方向を上下方向にする)
	g_pDynamicsWorld->setGravity(btVector3(0, -20, 0));
    
    //mikio 卵
    {
        btScalar MAIN_SPHERE_RADIUS = 0.2;

        btCollisionShape *main_Sphere = new btSphereShape(MAIN_SPHERE_RADIUS);
            //↑なんでアスタリスクがつくんだろう。

        btTransform trans;

        trans.setIdentity();// 位置姿勢行列の初期化
        trans.setOrigin(btVector3(0.1,MAIN_SPHERE_RADIUS+0.1,0.1));
        trans.setRotation(qrot);    // 四元数を行列に変換して姿勢行列に掛け合わせる
        
        g_MainBody = CreateRigidBody(1, trans, main_Sphere, 0, RX_COL_GROUP2, RX_COL_GROUND | RX_COL_GROUP1 | RX_COL_GROUP2);
        
            //すり抜け防止　ccdは動きが速くなる物体のみ設定する
        g_MainBody->setCcdSweptSphereRadius(MAIN_SPHERE_RADIUS);
        g_MainBody->setCcdMotionThreshold(0.05);
        
        g_MainBody->setUserIndex(1);
        g_MainBody->setFriction(0.5);
            
    }
    //mikio 足場
    {
        btCollisionShape *small_box_shape = new btBoxShape(btVector3(CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS));
        btCollisionShape *_box_shape = new btBoxShape(btVector3(2*CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS));
        btCollisionShape *_box_shape_ = new btBoxShape(btVector3(0.1, CUBE_HALF_EXTENTS, CUBE_HALF_EXTENTS));
        
        
        btTransform trans_1;
        trans_1.setIdentity();// 位置姿勢行列の初期化
        trans_1.setOrigin(btVector3(-1,0.3,0));
        trans_1.setRotation(qrot);
        scaffold_1 = CreateRigidBody(0, trans_1, small_box_shape, 0, RX_COL_GROUP2, RX_COL_GROUND | RX_COL_GROUP1 | RX_COL_GROUP2);
        scaffold_1->setFriction(1.0);
        
        scaffold_1->setUserIndex(2);
        
        btTransform trans_2;
        trans_2.setIdentity();// 位置姿勢行列の初期化
        trans_2.setOrigin(btVector3(-2,0.9,1));
        trans_2.setRotation(qrot);
        scaffold_2 = CreateRigidBody(0, trans_2, _box_shape, 0, RX_COL_GROUP2, RX_COL_GROUND | RX_COL_GROUP1 | RX_COL_GROUP2);
        scaffold_2->setFriction(1.0);
        
        //slider
        btTransform trans_3;
        trans_3.setIdentity();// 位置姿勢行列の初期化
        trans_3.setOrigin(btVector3(0,sliderHeight,0));
        trans_3.setRotation(qrot);
        scaffold_3 = CreateRigidBody(0, trans_3, _box_shape_, 0, RX_COL_GROUP2, RX_COL_GROUND | RX_COL_GROUP1 | RX_COL_GROUP2);
        scaffold_3->setFriction(1.0);
        scaffold_3->setRestitution(1);
        
        btTransform trans_4;
        trans_4.setIdentity();// 位置姿勢行列の初期化
        trans_4.setOrigin(btVector3(2,sliderHeight,0));
        trans_4.setRotation(qrot);
        scaffold_4 = CreateRigidBody(1, trans_4, _box_shape, 0, RX_COL_GROUP2, RX_COL_GROUND | RX_COL_GROUP1 | RX_COL_GROUP2);
        sliderr = new btSliderConstraint(*scaffold_3, *scaffold_4, trans_3, trans_4, 1);
        g_pDynamicsWorld->addConstraint(sliderr);
        scaffold_4->setRestitution(1);
        scaffold_4->applyCentralImpulse(btVector3(1,0,0));
        
        //scaffold_4->setUserIndex(2);
        
        
        btTransform trans_5;
        trans_5.setIdentity();// 位置姿勢行列の初期化
        trans_5.setOrigin(btVector3(3.0,sliderHeight,0));
        trans_5.setRotation(qrot);
        scaffold_5 = CreateRigidBody(0, trans_5, _box_shape_, 0, RX_COL_GROUP2, RX_COL_GROUND | RX_COL_GROUP1 | RX_COL_GROUP2);
        scaffold_5->setFriction(1.0);
        scaffold_5->setRestitution(1);
        
        btTransform trans_6;
        trans_6.setIdentity();// 位置姿勢行列の初期化
        trans_6.setOrigin(btVector3(-1,1.5,1));
        trans_6.setRotation(qrot);
        scaffold_6 = CreateRigidBody(0, trans_6, _box_shape, 0, RX_COL_GROUP2, RX_COL_GROUND | RX_COL_GROUP1 | RX_COL_GROUP2);
        scaffold_6->setFriction(1.0);
        
        btTransform trans_7;
        trans_7.setIdentity();// 位置姿勢行列の初期化
        trans_7.setOrigin(btVector3(-0,2.2,0.7));
        trans_7.setRotation(qrot);
        scaffold_7 = CreateRigidBody(0, trans_7, _box_shape, 0, RX_COL_GROUP2, RX_COL_GROUND | RX_COL_GROUP1 | RX_COL_GROUP2);
        scaffold_7->setFriction(1.0);
        
    }
    
    //3Dモデル読み込み
    {
        //utuwa
        rxPolygons poly;
        RxModel::Read("/Users/mikio_kamura/Library/Mobile Documents/com~apple~CloudDocs/CLASS/3_spring/bullet (物理)情報メディア実験/Mac環境用/iml_physics　壊れてない/bin/utuwa.obj", poly, Vec3(0,0,0), Vec3(0.3,0.3,0.3), Vec3(0,0,0));
        
        
        int vertex_count = (int)poly.vertices.size(); // 総頂点数
        int index_count = (int)poly.faces.size(); // 総ポリゴン数
        btScalar *vertices = new btScalar[vertex_count*3]; // 頂点座標を格納する配列
        int *indices = new int[index_count*3]; // ポリゴンを構成する頂点番号を格納する配列

        // 頂点座標の取り出し
        for(int i = 0; i < vertex_count; ++i){
            vertices[3*i] =   poly.vertices[i][0];
            vertices[3*i+1] = poly.vertices[i][1];
            vertices[3*i+2] = poly.vertices[i][2];
        }
        // ポリゴンを構成する頂点番号の取り出し
        for(int i = 0; i < index_count; ++i){
            indices[3*i]   = poly.faces[i][0];
            indices[3*i+1] = poly.faces[i][1];
            indices[3*i+2] = poly.faces[i][2];
        }
        
        int vertex_stride = 3*sizeof(btScalar);
        int index_stride = 3*sizeof(int);

        // 三角形メッシュ形状の作成
        btTriangleIndexVertexArray* tri_array = new btTriangleIndexVertexArray(index_count, indices, index_stride,
                                                                               vertex_count, vertices, vertex_stride);
        
        btGImpactMeshShape *utuwa = new btGImpactMeshShape(tri_array);
        utuwa->updateBound();
    
        owan_p = btVector3(2.0,4.2,0.0);
        
        btTransform owan_trans;
        owan_trans.setIdentity();
        owan_trans.setOrigin(owan_p);
        owan_trans.setRotation(qrot);
        owan = CreateRigidBody(0, owan_trans, utuwa, 0,RX_COL_GROUP2, RX_COL_GROUND | RX_COL_GROUP1 | RX_COL_GROUP2);
        
        //owan 底の衝突判定（clear判定）
        btCollisionShape *small_goal = new btBoxShape(btVector3(0.1, 0.01, 0.1));
        
        btTransform goal;
        goal.setIdentity();// 位置姿勢行列の初期化
        goal.setOrigin(owan_p);
        goal.setRotation(qrot);
        goal_rigid = CreateRigidBody(0, goal, small_goal, 0, RX_COL_GROUP2, RX_COL_GROUND | RX_COL_GROUP1 | RX_COL_GROUP2);
        goal_rigid->setFriction(1.0);
        
        goal_rigid->setUserIndex(2);
        
        
        
        
        //knife
        rxPolygons poly_knife;
        RxModel::Read("/Users/mikio_kamura/Library/Mobile Documents/com~apple~CloudDocs/CLASS/3_spring/bullet (物理)情報メディア実験/Mac環境用/iml_physics　壊れてない/bin/knife.obj", poly_knife, Vec3(0,0,0), Vec3(0.7,0.7,0.7), Vec3(0,0,0));
        
        
        int vertex_count_knife = (int)poly_knife.vertices.size(); // 総頂点数
        int index_count_knife = (int)poly_knife.faces.size(); // 総ポリゴン数
        btScalar *vertices_knife = new btScalar[vertex_count_knife*3]; // 頂点座標を格納する配列
        int *indices_knife = new int[index_count_knife*3]; // ポリゴンを構成する頂点番号を格納する配列

        // 頂点座標の取り出し
        for(int i = 0; i < vertex_count_knife; ++i){
            vertices_knife[3*i] =   poly_knife.vertices[i][0];
            vertices_knife[3*i+1] = poly_knife.vertices[i][1];
            vertices_knife[3*i+2] = poly_knife.vertices[i][2];
        }
        // ポリゴンを構成する頂点番号の取り出し
        for(int i = 0; i < index_count_knife; ++i){
            indices_knife[3*i]   = poly_knife.faces[i][0];
            indices_knife[3*i+1] = poly_knife.faces[i][1];
            indices_knife[3*i+2] = poly_knife.faces[i][2];
        }
        
        int vertex_stride_knife = 3*sizeof(btScalar);
        int index_stride_knife = 3*sizeof(int);

        // 三角形メッシュ形状の作成
        btTriangleIndexVertexArray* tri_array_knife = new btTriangleIndexVertexArray(index_count_knife, indices_knife, index_stride_knife,
                                                                               vertex_count_knife, vertices_knife, vertex_stride_knife);
        
        btGImpactMeshShape *knife_shape = new btGImpactMeshShape(tri_array_knife);
        knife_shape->updateBound();
        
        board_p = btVector3(0,2.0,0.0);
        
        btTransform knife_trans;
        knife_trans.setIdentity();
        knife_trans.setOrigin(board_p + btVector3(0,1.0,0));
        knife_trans.setRotation(qrot);
        knife = CreateRigidBody(0, knife_trans, knife_shape, 0,RX_COL_GROUP2, RX_COL_GROUND | RX_COL_GROUP1 | RX_COL_GROUP2);
        
        
        //まな板　chopping board
        rxPolygons poly_board;
        RxModel::Read("/Users/mikio_kamura/Library/Mobile Documents/com~apple~CloudDocs/CLASS/3_spring/bullet (物理)情報メディア実験/Mac環境用/iml_physics　壊れてない/bin/choppingBoard.obj", poly_board, Vec3(0,0,0), Vec3(0.7,0.7,0.7), Vec3(0,0,0));
        
        
        int vertex_count_board = (int)poly_board.vertices.size(); // 総頂点数
        int index_count_board = (int)poly_board.faces.size(); // 総ポリゴン数
        btScalar *vertices_board = new btScalar[vertex_count_board*3]; // 頂点座標を格納する配列
        int *indices_board = new int[index_count_board*3]; // ポリゴンを構成する頂点番号を格納する配列

        // 頂点座標の取り出し
        for(int i = 0; i < vertex_count_board; ++i){
            vertices_board[3*i] =   poly_board.vertices[i][0];
            vertices_board[3*i+1] = poly_board.vertices[i][1];
            vertices_board[3*i+2] = poly_board.vertices[i][2];
        }
        // ポリゴンを構成する頂点番号の取り出し
        for(int i = 0; i < index_count_board; ++i){
            indices_board[3*i]   = poly_board.faces[i][0];
            indices_board[3*i+1] = poly_board.faces[i][1];
            indices_board[3*i+2] = poly_board.faces[i][2];
        }
        
        int vertex_stride_board = 3*sizeof(btScalar);
        int index_stride_board = 3*sizeof(int);

        // 三角形メッシュ形状の作成
        btTriangleIndexVertexArray* tri_array_board = new btTriangleIndexVertexArray(index_count_board, indices_board, index_stride_board,
                                                                               vertex_count_board, vertices_board, vertex_stride_board);
        
        btGImpactMeshShape *board_shape = new btGImpactMeshShape(tri_array_board);
        board_shape->updateBound();
        
        btTransform board_trans;
        board_trans.setIdentity();
        board_trans.setOrigin(board_p);
        board_trans.setRotation(qrot);
        board = CreateRigidBody(0, board_trans, board_shape, 0,RX_COL_GROUP2, RX_COL_GROUND | RX_COL_GROUP1 | RX_COL_GROUP2);

        
    }
    
    
    
	SetRigidBodies();

}

/*!
 * 設定したBulletの剛体オブジェクト，ワールドの破棄
 */
void CleanBullet(void)
{
	// 剛体オブジェクトの破棄
	for(int i = g_pDynamicsWorld->getNumCollisionObjects()-1; i >= 0; --i){
		btCollisionObject* obj = g_pDynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if(body && body->getMotionState()){
			delete body->getMotionState();
		}
		g_pDynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	// 形状の破棄
	for(int j = 0; j < (int)g_vCollisionShapes.size(); ++j){
		btCollisionShape* shape = g_vCollisionShapes[j];
		g_vCollisionShapes[j] = 0;
		delete shape;
	}
	g_vCollisionShapes.clear();
    
    //拘束条件の破棄　練習問題１
    for(int i = g_pDynamicsWorld->getNumConstraints()-1; i>=0; i--){
        btTypedConstraint* constraint = g_pDynamicsWorld->getConstraint(i);
        g_pDynamicsWorld->removeConstraint(constraint);
        delete constraint;
    }
    
    g_pManyBody.clear();
	// ワールド破棄
	delete g_pDynamicsWorld;
}


//-----------------------------------------------------------------------------
// MARK:描画関数
//-----------------------------------------------------------------------------
/*!
 * 透視投影変換
 */
void Projection(void)
{
	gluPerspective(RX_FOV, (double)g_iWinW/(double)g_iWinH, 0.2f, 1000.0f);
}

/*!
 * Bulletのオブジェクトの描画シーン描画
 */
void DrawBulletObjects(void)
{
	static const GLfloat difr[] = { 1.0, 0.4, 0.4, 1.0 };	// 拡散色 : 赤
	static const GLfloat difg[] = { 0.4, 0.6, 0.4, 1.0 };	// 拡散色 : 緑
	static const GLfloat difb[] = { 0.4, 0.4, 1.0, 1.0 };	// 拡散色 : 青
	static const GLfloat spec[] = { 0.3, 0.3, 0.3, 1.0 };	// 鏡面反射色
	static const GLfloat ambi[] = { 0.1, 0.1, 0.1, 1.0 };	// 環境光

	glEnable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	glDisable(GL_COLOR_MATERIAL);

	glMaterialfv(GL_FRONT, GL_SPECULAR, spec);
	glMaterialfv(GL_FRONT, GL_AMBIENT,  ambi);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0f);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glColor3f(0.0, 0.0, 1.0);

	glDisable(GL_CULL_FACE);

	if(g_pDynamicsWorld){
		btScalar m[16];
		btMatrix3x3	rot;
		rot.setIdentity();

		// Bulletワールドから剛体オブジェクト情報を取得してOpenGLで描画
		const int n = g_pDynamicsWorld->getNumCollisionObjects();	// オブジェクト数の取得
		for(int i = 0; i < n; ++i){

			// btCollisionObject → btRigidBodyへのキャストで剛体オブジェクトを取得
			btCollisionObject* obj = g_pDynamicsWorld->getCollisionObjectArray()[i];

			// 形状取得
			btCollisionShape* shape = obj->getCollisionShape();
			int shapetype = shape->getShapeType();

			if(shapetype == SOFTBODY_SHAPE_PROXYTYPE){
				btSoftBody* body = btSoftBody::upcast(obj);

				glMaterialfv(GL_FRONT, GL_DIFFUSE, difb);

				// draw a softbody
				DrawBulletSoftBody(body);
			} 
			else{
				btRigidBody* body = btRigidBody::upcast(obj);//ここでキャストを行なっている
				if(body && body->getMotionState()){
					// btRigidBodyからMotion Stateを取得して，OpenGLの変換行列として位置・姿勢情報を得る
					btDefaultMotionState* ms = (btDefaultMotionState*)body->getMotionState();
                    //?(btDefaultMotionState*)で何をしているのかわからん、、方の変更か。upcastをしている、出なければgetMoitionState()ができないのかな
                    //というわけでもなさそう、、謎
                    
					ms->m_graphicsWorldTrans.getOpenGLMatrix(m);
                    //? ->と.の使い分けがわからない  ポインタのメンバ関数にアクセスしてるか、そのものにアクセスしているかの違いらしかった
                    //msはbtDefaultMotionStateのインスタンス？要素？btDefaultMotionStateはclassではなくstructure。。。
                    //getOpenGLMatrix()は、btTransformのメンバ関数
                    //m_garphicsWorldTransは、btDefaultMotionStateで定義されていて、型がbtTransformである、変数の名前。
                    
                    //?なんでbtDefaultMotionStateが噛んでいるのかわからない。そのままbody->getMotionState().getOpenGLMatrix();アjだめなのか
					rot = ms->m_graphicsWorldTrans.getBasis();
				} else{
					obj->getWorldTransform().getOpenGLMatrix(m);
					rot = obj->getWorldTransform().getBasis();
				}
                
				if(body && body->getInvMass() > RX_FEQ_EPS ){
                    // Dynamicボディ
                    if(body->isActive()){
                        glMaterialfv(GL_FRONT, GL_DIFFUSE, difb);
                    }else{
                        glMaterialfv(GL_FRONT, GL_DIFFUSE, difr);
                    }
				} else{	// Kinematicボディの場合は緑で描画
					glMaterialfv(GL_FRONT, GL_DIFFUSE, difg);
				}

				btVector3 world_min, world_max;
				g_pDynamicsWorld->getBroadphase()->getBroadphaseAabb(world_min, world_max);

				glPushMatrix();
				glMultMatrixf(m);

				// 形状描画
				DrawBulletShape(shape, world_min, world_max);

				glPopMatrix();
                
            }
		}
	}

}


/*!
 * シーン描画
 */
void RenderScene(void* x = 0)
{
	// 光源設定
	glLightfv(GL_LIGHT0, GL_POSITION, RX_LIGHT0_POS);

	DrawBulletObjects();
}


/*!
 * 描画関数
 */
void Display(void)
{
	// フレームバッファとデプスバッファをクリアする
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();
	g_tbView.Apply();	// マウスによる回転・平行移動の適用

	// シャドウマップを使って影付きでオブジェクト描画
	Vec3 light_pos = Vec3(RX_LIGHT0_POS[0], RX_LIGHT0_POS[1], RX_LIGHT0_POS[2]);
	rxFrustum light = CalFrustum(90, 0.02, 50.0, g_iShadowMapSize, g_iShadowMapSize, light_pos, Vec3(0.0));
	g_ShadowMap.RenderSceneWithShadow(light, RenderScene, 0);

    glPointSize(5);
    glDisable(GL_LIGHTING);
    glBegin(GL_POINTS);
    
    //frameRate++;
    
    int num_manifolds = g_pDynamicsWorld->getDispatcher()->getNumManifolds(); // 衝突候補のペアの数
    Onsomething = 0;
    clear = 0;
      for(int i = 0; i < num_manifolds; ++i){    // 各ペアを調べていく
          // 衝突点を格納するためのキャッシュ(manifold)から情報を取得
          btPersistentManifold* manifold = g_pDynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
          btCollisionObject* obA = const_cast<btCollisionObject*>(manifold->getBody0()); // 衝突ペアのうちのオブジェクトA
          btCollisionObject* obB = const_cast<btCollisionObject*>(manifold->getBody1()); // 衝突ペアのうちのオブジェクトB

          // 各オブジェクトのユーザーインデックス(練習問題6で使います)
          int user_idx0 = obA->getUserIndex();
          int user_idx1 = obB->getUserIndex();

          if(user_idx0 == 1 || user_idx1 == 1) Onsomething = 1;
          if(user_idx0 == 2 || user_idx1 == 2){
              clear = 1;
          }
      }
    glEnd();

    //mikiodisplay
    {
        g_tbView.CalLocalPos(eye_pos, init_pos);
        g_tbView.CalLocalRot(eye_dir, init_dir);
        
        btVector3 eyedir(eye_dir[0], eye_dir[1],eye_dir[2]);
        btVector3 eyepos(eye_pos[0], eye_pos[1],eye_pos[2]);
        
        btVector3 suityoku = eyedir.cross(btVector3(0,1,0)).normalize();
        
       if(body_track_eye == 1){
            MoveCameraWithRigid(g_tbView, g_MainBody);
        }
        
//        if(jumping == 1){
//            jumping = -1;
//        }
        if(jumping == 0){
            //ジャンプを離した時の処理
            g_MainBody->applyCentralImpulse(-g_MainBody->getLinearVelocity() * g_MainBody->getMass() * 0.5);
            
            jumping = -1;
        }
        //right -> l
        if(rightward == 1){
            //impulse
            right_cnt++;
            if(right_cnt == 1){
                g_MainBody->applyCentralImpulse(suityoku * 3.0);
            }
            rightward = -1;
        }
        if(rightward == 0){
            g_MainBody->applyCentralImpulse(suityoku * -3.0);
            right_cnt = 0;
            rightward = -1;
        }
        //left -> j
        if(leftward == 1){
            //impulse
            left_cnt++;
            if(left_cnt == 1 && jumping != 1){
                g_MainBody->applyCentralImpulse(suityoku * -3.0);
            }
            leftward = -1;
        }
        if(leftward == 0){
            g_MainBody->applyCentralImpulse(suityoku * 3.0);
            left_cnt = 0;
            leftward = -1;
        }
        //forward -> i
        if(forwardd == 1 && jumping != 1){
            //impulse
            forward_cnt++;
            if(forward_cnt == 1){
                g_MainBody->applyCentralImpulse(eyedir * 3);
            }
            forwardd = -1;
        }
        if(forwardd == 0){
            g_MainBody->applyCentralImpulse(eyedir * -3);
            forward_cnt = 0;
            forwardd = -1;
        }
        //backward -> m
        if(backwardd == 1 && jumping != 1){
            //impulse
            backward_cnt++;
            if(backward_cnt == 1){
                g_MainBody->applyCentralImpulse(eyedir * -3);
            }
            backwardd = -1;
        }
        if(backwardd == 0){
            g_MainBody->applyCentralImpulse(eyedir * 3);
            backward_cnt = 0;
            backwardd = -1;
        }
        //加速
        if(accel == 1){
            //impulse
            accel_cnt++;
            if(accel_cnt == 1 && (jumping == 0 || jumping == -1)){
                g_MainBody->applyCentralImpulse( g_MainBody->getLinearVelocity());
            }
            accel = -1;
        }
        if(accel == 0){
            //g_MainBody->applyCentralImpulse(eyedir * 3);
            accel_cnt = 0;
            accel = -1;
        }

        
    }
    if(clear != 1){
        glColor3d(0.0, 0.0, 1.0);
        DrawString("press 's' to start or stop.", g_iWinW, g_iWinH, 20, 20);
        glColor3d(1.0, 0.0, 0.0);
        DrawString("aim for the owan!!!", g_iWinW, g_iWinH, 20, 50);
    }
    if(clear == 1){
        glColor3d(1.0, 0.0, 0.0);
        DrawString("clear! you are safe now!!!", g_iWinW, g_iWinH, 50, 50);
    }
        
	glPopMatrix();
    
	glutSwapBuffers();
}


/*!
 * リサイズイベント処理関数
 * @param[in] w,h キャンバスサイズ
 */
void Resize(int w, int h)
{
	glViewport(0, 0, w, h);
	g_tbView.SetRegion(w, h);

	g_iWinW = w;
	g_iWinH = h;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(RX_FOV, (float)w/(float)h, 0.01f, 20.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

/*!
 * マウスイベント処理関数
 * @param[in] button マウスボタン(GLUT_LEFT_BUTTON,GLUT_MIDDLE_BUTTON,GLUT_RIGHT_BUTTON)
 * @param[in] state マウスボタンの状態(GLUT_UP, GLUT_DOWN)
 * @param[in] x,y マウス座標(スクリーン座標系)
 */
void Mouse(int button, int state, int x, int y)
{
	if(x < 0 || y < 0) return;
	g_iMouseButton = button;
	int mod = glutGetModifiers();	// SHIFT,CTRL,ALTの状態取得

	if(button == GLUT_LEFT_BUTTON){
		if(state == GLUT_DOWN){
			g_pPickNode = 0;
			double ray_from0[3], ray_to0[3];
			double init_pos[3] = { 0, 0, 0 };
			g_tbView.CalLocalPos(ray_from0, init_pos);

			g_tbView.GetRayTo(x, y, RX_FOV, ray_to0);

			btVector3 ray_from = btVector3(ray_from0[0], ray_from0[1], ray_from0[2]);
			btVector3 ray_to = btVector3(ray_to0[0], ray_to0[1], ray_to0[2]);

			btCollisionWorld::ClosestRayResultCallback ray_callback(ray_from, ray_to);
			g_pDynamicsWorld->rayTest(ray_from, ray_to, ray_callback);

			if(ray_callback.hasHit()){
				const btCollisionObject* obj = ray_callback.m_collisionObject;

				// 光線と衝突した剛体
				btRigidBody* body = const_cast<btRigidBody*>(btRigidBody::upcast(obj));

				// 衝突点座標(ジョイントになる位置座標)
				btVector3 picked_pos = ray_callback.m_hitPointWorld;

				if(body){
					if(!(body->isStaticObject() || body->isKinematicObject())){
						g_pPickBody = body;
						g_vPickPos = picked_pos;

						// 選択された剛体の座標系でのピック位置
						btVector3 local_pos = body->getCenterOfMassTransform().inverse()*picked_pos;

						g_pPickBody->setActivationState(DISABLE_DEACTIVATION); // 必要！

						if(g_pPickConstraint){
							g_pDynamicsWorld->removeConstraint(g_pPickConstraint);
							delete g_pPickConstraint;
						}
						g_pPickConstraint = new btPoint2PointConstraint(*body, local_pos);
						g_pDynamicsWorld->addConstraint(g_pPickConstraint, true);

						g_pPickConstraint->m_setting.m_impulseClamp = 30.0;
						g_pPickConstraint->m_setting.m_tau = 0.001f;

						g_fPickDist = (g_vPickPos-ray_from).length();
					}
				}
				else{
					// 光線と衝突したbtSoftBody
					btSoftBody* body = const_cast<btSoftBody*>(btSoftBody::upcast(obj));
					btSoftBody::sRayCast res;
					body->rayTest(ray_from, ray_to, res);
					if(res.fraction < 1.0){
						btVector3 impact = ray_from+(ray_to-ray_from)*res.fraction;
						cout << impact << endl;
						if(res.feature == btSoftBody::eFeature::Face){
							btSoftBody::Face& face = res.body->m_faces[res.index];

							// 衝突点に最も近いノードを探索
							btSoftBody::Node* node = face.m_n[0];
							for(int i = 1; i < 3; ++i){
								if((node->m_x-impact).length2() >(face.m_n[i]->m_x-impact).length2()){
									node = face.m_n[i];
								}
							}
							g_pPickNode = node;
							g_fPickDist = (g_pPickNode->m_x - ray_from).length();
						}
					}
				}
			}

			if(!g_pPickConstraint && !g_pPickNode){
				g_tbView.Start(x, y, mod+1);
			}
		}
		else if(state == GLUT_UP){
			if(g_pPickConstraint){
				g_pDynamicsWorld->removeConstraint(g_pPickConstraint);
				delete g_pPickConstraint;
				g_pPickConstraint = 0;
				g_pPickBody = 0;
			} else if(g_pPickNode){
				g_pPickNode = 0;
			} else{
				g_tbView.Stop(x, y);
			}
		}
	}

}

/*!
 * モーションイベント処理関数(マウスボタンを押したままドラッグ)
 * @param[in] x,y マウス座標(スクリーン座標系)
 */
void Motion(int x, int y)
{
	if(g_iMouseButton == GLUT_LEFT_BUTTON){
		if(g_pPickConstraint || g_pPickNode){
			double ray_from0[3], ray_to0[3];
			double init_pos[3] = { 0, 0, 0 };
			g_tbView.CalLocalPos(ray_from0, init_pos);
			g_tbView.GetRayTo(x, y, RX_FOV, ray_to0);

			btVector3 ray_from = btVector3(ray_from0[0], ray_from0[1], ray_from0[2]);
			btVector3 new_ray_to = btVector3(ray_to0[0], ray_to0[1], ray_to0[2]);

			btVector3 dir = new_ray_to-ray_from;
			dir.normalize();

			btVector3 new_pivot = ray_from+dir*g_fPickDist;

			if(g_pPickConstraint){
				g_pPickConstraint->setPivotB(new_pivot);
			} else if(g_pPickNode){
				g_pPickNode->m_f += (new_pivot-g_pPickNode->m_x)*10.0;
			}

			g_vPickPos = new_pivot;
		}
		else{
			g_tbView.Motion(x, y);
		}
	}
	glutPostRedisplay();
}

/*!
 * アイドルイベント処理関数(CPUが暇なときに実行)
 */
void Idle(void)
{
	glutPostRedisplay();
}

/*!
 * タイマーイベント処理関数(ある時間間隔で実行)
 */
void Timer(int value)
{
	if(g_bIdle && g_pDynamicsWorld){
		// シミュレーションを1ステップ進める
		int numstep = g_pDynamicsWorld->stepSimulation(DT, 1);
	}
    t++;
    if( t%200 == 0){
     scaffold_4->applyCentralImpulse(scaffold_4->getLinearVelocity()*0.1);
    }
    g_MainBody->setAngularVelocity(btVector3(0,0,0));
    
//    if(rightward == 1){
//        g_MainBody->setAngularVelocity(btVector3(0,1,0)*10);
//    }
    
	glutPostRedisplay();
	glutTimerFunc(DT*1000 , Timer , 0);




}

void SpecialKey(int key, int x, int y){
  
    glutPostRedisplay();
}


/*!
 * キーボードイベント処理関数
 * @param[in] key キーの種類
 * @param[in] x,y キーが押されたときのマウス座標(スクリーン座標系)
 */
void Keyboard(unsigned char key, int x, int y)
{
	switch(key){
	case '\033':  // '\033' は ESC の ASCII コード
		CleanBullet();
		exit(0);
            
	case 's':	// アニメーション切り替え
		g_bIdle = !g_bIdle;
        if(started == -1){
            
        }
		break;

	case ' ':	// 1ステップだけ進める
		Timer(0);
		break;

	case 'r':	// ワールドリセット
		CleanBullet();
		InitBullet();
		break;
            
    case 'e':
        {
            body_track_eye = 1;
            break;
        }
    case 'd':
        {
            if(jumping != 0){
                g_MainBody->applyCentralImpulse(-g_MainBody->getLinearVelocity() * g_MainBody->getMass());
            }
            break;
        }
    case 'k'://jump ジャンプ
        {
            jumping = 1;
            if(Onsomething == 1){//接地面があったら
                g_MainBody->applyCentralImpulse(btVector3(0,1.0,0)*10);
            }
            break;
        }
    case 'l'://右
        {
            rightward = 1;
            break;
        }
    case 'j'://左
        {
            leftward = 1;
            break;
        }
    case 'i'://前
        {
            forwardd = 1;
            break;
        }
    case 'm'://後ろ
        {
            backwardd = 1;
            break;
        }
    case 'n'://加速
        {
            accel = 1;
            break;
        }
	default:
		break;
	}
}

void KeyboardUp(unsigned char key, int x, int y){
    switch(key){
    case 'e':
        {
            body_track_eye = 0;
            break;
            
        }
    case 'k'://jump
    {
        jumping = 0;
        break;
    }
    case 'l'://右
        {
            rightward = 0;
            break;
        }
    case 'j'://左
        {
            leftward = 0;
            break;
        }
    case 'i'://右
        {
            forwardd = 0;
            break;
        }
    case 'm'://左
        {
            backwardd = 0;
            break;
        }
    case 'n'://後ろ
        {
            accel = 0;
            break;
        }
    }
}


/*!
 * OpenGLの初期化
 */
void InitGL(void)
{
	// OpenGLのバージョンチェック
	printf("OpenGL Ver. %s\n", glGetString(GL_VERSION));
	
	// 背景色
	glClearColor(0.8, 0.8, 0.9, 1.0);

	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// 光源の初期設定
	glEnable(GL_LIGHTING);
	glLightfv(GL_LIGHT0, GL_DIFFUSE,  RX_LIGHT_DIFF);
	glLightfv(GL_LIGHT0, GL_SPECULAR, RX_LIGHT_SPEC);
	glLightfv(GL_LIGHT0, GL_AMBIENT,  RX_LIGHT_AMBI);
	glLightfv(GL_LIGHT0, GL_POSITION, RX_LIGHT0_POS);

	glShadeModel(GL_SMOOTH);

	glEnable(GL_AUTO_NORMAL);
	glEnable(GL_NORMALIZE);

	// 視点の初期化
	g_tbView.SetScaling(-7.0);
	g_tbView.SetTranslation(0.0, -2.0);

	// シャドウマップ初期化
	g_ShadowMap.InitShadow(g_iShadowMapSize, g_iShadowMapSize);

	// Bullet初期化
	InitBullet();
}


/*!
 * メインルーチン
 * @param[in] argc コマンドライン引数の数
 * @param[in] argv コマンドライン引数
 */
int main(int argc, char *argv[])
{
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(g_iWinW, g_iWinH);
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow(argv[0]);

	// イベントハンドラの設定
	glutDisplayFunc(Display);
	glutReshapeFunc(Resize);
	glutMouseFunc(Mouse);
	glutMotionFunc(Motion);
	glutKeyboardFunc(Keyboard);
	//glutIdleFunc(Idle);
    glutKeyboardUpFunc(KeyboardUp);
	glutTimerFunc(DT*1000, Timer, 0);
	g_bIdle = false;

	InitGL();
    
    glutSpecialFunc(SpecialKey);
	glutMainLoop();
    //Loopの下でいいのかな。
    glutSpecialFunc(SpecialKey);
    
	CleanBullet();

	return 0;
}

