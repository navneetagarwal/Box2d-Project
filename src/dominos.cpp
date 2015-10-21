/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 251 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * 
 */

#define PI 3.14159

#include "cs251_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
  #include <GLUT/glut.h>
#else
  #include "GL/freeglut.h"
#endif

#include <cstring>
 using namespace std;

#include "dominos.hpp"
 /*! \var a
     * \brief Used for the initial x coordinate of a component
     */ 
  /*! \var b
     * \brief Used for the initial y coordinate of a component
     */ 
  /*! \var i
     * \brief Used for the initial x coordinate of a component
     */ 
  /*! \var j
     * \brief Used for the initial y coordinate of a component
     */ 
  /*! \var b2Body* b1
* \brief It is the pointer to b2Body
* It is the pointer to ground
*/
/*! \var b2EdgeShape shape
* \brief An object of type b2EdgeShape. 
* It creates the edge for the horizontal ground
  Defined from -90 to +90
*/
/*! \var b2BodyDef bd
* \brief An object of type b2BodyDef
* It creates the bodies for the Dominos, Ground, Horizontal shelf
Value different at different positions
*/
/*! \var b2PolygonShape shape1
* \brief An object of type b2PolygonShape. 
* It creates the shape of the bodies
Value different at different positions
*/
/*! \var b2FixtureDef fd
* \brief An object of type b2FixtureDef. 
* It creates the shape of the dominos at various positions
*/
/*! \var b2Body* b4
* \brief A pointer to the various bodies 
* It creates the constructed fixture
*/
/*! \var b2Body* b2
* \brief A pointer to the mills of the mill 
* Helps creating the constructed fixture
*/
/*! \var b2RevoluteJointDef jd
* \brief An object of type b2RevoluteJointDef.
* It creates the joint for pendulum,pulley, the joint for the wedge and plank and for the mills and ground also.
Value different at different positions
*/
/*! \var b2Vec2 anchor
* \brief An object of type b2Vec2. 
* It creates the anchor for the RevoluteJoints to be created
Value different at different positions
*/
/*! \var b2Body* spherebody
* \brief A pointer to the constructed spheres 
* It creates the spheres
*/
/*! \var b2Body* ground
* \brief Another pointer to various bodies like the rectangular part of the see saw
* It creates the bodies.
*/
/*! \var b2CircleShape circle
* \brief An object of type b2CircleShape. 
* It creates the ciruclar shape for the spheres
Value different at different positions
*/
/*! \var b2FixtureDef ballfd
* \brief An object of type b2FixtureDef. 
* Defines the fixture for the various balls used.
Value different at different positions
*/
/*! \var b2BodyDef ballbd
* \brief An object of type b2BodyDef. 
* It defines the body of the balls used by defining static or dynamic and by giving the position
Value different at different positions
*/
/*! \var b2BodyDef *bd1
* \brief A definition for some balls as well as some pulleys
* Value different at different positions
*/
/*! \var b2FixtureDef *fd1
* \brief Defines a fixture of a side of the pulley and the container
* It helps in creation of a side for the open box
*/
/*! \var b2PolygonShape bs1
* \brief An object of type b2PolygonShape. 
* It defines the shape of a fixture
*/
/*! \var b2PolygonShape bs2
* \brief An object of type b2PolygonShape. 
* It defines the shape of a fixture
*/
/*! \var b2PolygonShape bs3
* \brief An object of type b2PolygonShape. 
* It defines the shape of a fixture
*/
/*! \var b2FixtureDef *fd2
* \brief An object of type b2FixtureDef 
* It points to the fixture for the side of a pulley,plank on the wedge and the containers
*/
/*! \var b2FixtureDef *fd3
* \brief An object of type b2FixtureDef 
* It points to the fixture for the side of a pulley,plank on the wedge and the containers
*/
/*! \var b2FixtureDef *fd4
* \brief An object of type b2FixtureDef 
* It points to the fixture for the side of a pulley,plank on the wedge and the containers
*/
/*! \var b2FixtureDef *fd5
* \brief An object of type b2FixtureDef 
* It points to the fixture for the side of a pulley,plank on the wedge and the containers
*/
/*! \var b2FixtureDef *fd6
* \brief An object of type b2FixtureDef 
* It points to the fixture for the side of a pulley,plank on the wedge and the containers
*/
/*! \var b2Body* box1
* \brief An object of type b2Body. 
* Defines a body which contains many fixtures defined inside it.Pulley containers and Snooker table
*/
/*! \var b2Body* box2
* \brief An object of type b2Body. 
* Defines a body which contains many fixtures defined inside it.Pulley containers and Snooker table
*/
/*! \var b2PulleyJointDef* myjoint
* \brief A pointer to the pulley joint. 
* It creates the pulley joint
*/
/*! \var float32 ratio
* \brief An object of type float32. 
* It defines the ratio of pulley anchor and joint
Value = 1
*/
/*! \var b2PolygonShape shape2
* \brief An object of type b2PolygonShape. 
* It creates the shape of the horizontal plank,vertical rotors etc.
Value different at different positions
*/
/*! \var b2BodyDef bd2
* \brief An object of type b2BodyDef. 
* Defines the bodies of the plank and the rotors, the flask.
*/
/*! \var b2RevoluteJointDef jointDef
* \brief An object of type b2RevoluteJointDef. 
* It creates the revolve joint for the criss cross chain structure ,the newton's cradle and the rotors.
*/
/*! \var b2Body* sbody
* \brief A pointer to bodies such as the sloping plank and the triangular wedge on seesaw
* It helps in the creation of these bodies
*/
/*! \var b2PolygonShape poly
* \brief An object of type b2PolygonShape. 
* Used to create shapes which are polygonal like triangular wedge and slanted paths.
*/
/*! \var b2FixtureDef wedgefd
* \brief An object of type b2FixtureDef. 
* It creates the fixture for the triangular wedge and the sloping planks
*/
/*! \var b2BodyDef wedgebd
* \brief An object of type b2BodyDef. 
* It creates the shape of the of the triangular wedge ,the sloping planks
*/
/*! \var b2PolygonShape plank
* \brief An object of type b2PolygonShape. 
* It creates the shape of the planks ,containers and seesaw.
Value different at different positions
*/
/*! \var width
     * \brief Used for the width of each mill
     */ 
/*! \var heightdiff
     * \brief Used for making four mills at the given height difference
     */ 
   /*! \var b2FixtureDef millfd
* \brief An object of type b2FixtureDef. 
* It creates the fixture for the mills
*/  
 /*! \var b2FixtureDef millfd2
* \brief An object of type b2FixtureDef. 
* It creates the fixture for the mills
*/
/*! \var millx
     * \brief Used for the x-coordinate of the mill
     */ 
 /*! \var milly
 * \brief Used for the y- coordinate of each mill
 */ 
 /*! \var b2BodyDef bx
* \brief An object of type b2BodyDef. 
* It creates the shape of the mills
*/
/*! \var conveyorleft
     * \brief Initial x coordinate of the conveyor belt
     */ 
/*! \var conveyorup
     * \brief Initial y coordinate of the conveyor belt
     */ 
/*! \var links
     * \brief The number of chains
     */  
/*! \var speed
     * \brief Velocity of the discs
     */ 
/*! \var frction
     * \brief Value of friction between the discs
     */ 
/*! \var speed
     * \brief Velocity of the discs
     */ 
/*! \var leftup
     * \brief leftmost uppermost chain link
     */ 
/*! \var rightup
     * \brief rightmost uppermost chain link
     */ 
/*! \var leftdown
     * \brief leftmost bottommost chain link
     */ 
/*! \var rightdown
     * \brief rightmost bottommost chain link
     */ 
/*! \var chainpiece
     * \brief defines the shape for each link
     */
/*! \var chainfix
     * \brief Defines the fixture for each link
     */ 
/*! \var lastLink
     * \brief Pointer to the previous link
     */ 
/*! \var chainJoint
     * \brief RevoluteJoint between two consecutive joints
     */ 
  /*! \var chainJoint2
     * \brief RevoluteJoint between two consecutive joints
     */    
     /*! \var chainJoint3
     * \brief RevoluteJoint between two consecutive joints
     */ 
/*! \var nextLink
     * \brief Pointer to the next link
     */ 
/*! \var shiftup
     * \brief Distance between the lower and upper links
     */ 
/*! \var b2FixtureDef ball
* \brief An object of type b2FixtureDef. 
* Defines the fixture of ball 
*/
/*! \var b2BodyDef ballBody
* \brief An object of type b2BodyDef. 
* Defines the body of balls starting newton's cradle,the two balls on left and the one falling on criss cross structure
*/

/*! \var b2Body* sbody00
* \brief Pointer to body of the first part of the upper row in the criss cross structure
*/
/*! \var poly00
* \brief Defines the shape of the first part of the upper row in the criss cross structure
*/
/*! \var wedgefd00
* \brief Defines the fixture of the first part of the upper row in the criss cross structure
*/
/*! \var wedgebd00
* \brief Defines the body of the first part of the upper row in the criss cross structure
*/
/*! \var Joint00
* \brief Defines the joint of the first part of the upper row in the criss cross structure
*/
float a;
float b;
float i;
float j;
float width;
float heightdiff;
float millx;
float milly;
double conveyorleft;
double conveyorup;
int links;
double speed;
double frction;
b2Body* leftup;
b2Body* rightup;
b2Body* leftdown;
b2Body* rightdown;
b2PolygonShape chainpiece;
b2FixtureDef chainfix;
b2Body *lastLink;
b2RevoluteJointDef chainJoint;
b2Body* nextLink;
double shiftup ;
b2FixtureDef ball;
b2BodyDef ballBody;
b2Body* sbody00;
b2PolygonShape poly00;
b2FixtureDef wedgefd00;
b2BodyDef wedgebd00;
b2RevoluteJointDef Joint00;
b2RevoluteJointDef chainJoint2;
 b2RevoluteJointDef chainJoint3;
b2Body* b1; 
b2EdgeShape shape; 
b2BodyDef bd;
b2PolygonShape shape1;
b2FixtureDef fd;
b2Body* b4;
b2Body* b2;
b2RevoluteJointDef jd;
b2Vec2 anchor;
b2Body* spherebody;
b2Body* ground;
b2CircleShape circle;
b2FixtureDef ballfd;
b2BodyDef ballbd;
b2BodyDef *bd1;
b2FixtureDef *fd1;
b2PolygonShape bs1;
b2PolygonShape bs2;
b2PolygonShape bs3;
b2FixtureDef *fd2;
b2FixtureDef *fd3;
b2FixtureDef *fd4;
b2FixtureDef *fd5;
b2FixtureDef *fd6;
b2Body* box1;
b2Body* box2 ;
b2PulleyJointDef* myjoint;
float32 ratio;
b2PolygonShape shape2;
b2BodyDef bd2;
b2RevoluteJointDef jointDef;
b2Body* sbody;
b2PolygonShape poly;
b2FixtureDef wedgefd;
b2BodyDef wedgebd;
b2PolygonShape plank;
b2FixtureDef millfd;
b2FixtureDef millfd2;
b2BodyDef bx;

namespace cs251
{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   */ 
//Variables
   dominos_t::dominos_t()
   {
    //Ground
    /*! \var b1 
     * \brief pointer to the body ground 
     */ 

     b2Body* b1;  
     {

      b2EdgeShape shape; 
      shape.Set(b2Vec2(-1000.0f, 0.0f), b2Vec2(1000.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }


/**
 * ... text ...
 */
    {     //The open box
      float a =10.0f;
      float b = 70.0f;
      {
        b2BodyDef *bd = new b2BodyDef;
      //bd->type = b2_dynamicBody;
        bd->position.Set(0,5);
        bd->fixedRotation = true;

        b2FixtureDef *fd1 = new b2FixtureDef;
        fd1->density = 10.0;
        fd1->friction = 0.5;
        fd1->restitution = 1.f;
        fd1->shape = new b2PolygonShape;
        b2PolygonShape bs1;
        bs1.SetAsBox(6,0.1, b2Vec2(a+0.0f,b+7.5f), 0);
        fd1->shape = &bs1;
        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = 10.0;
        fd2->friction = 0.5;
        fd2->restitution = 1.f;
        fd2->shape = new b2PolygonShape;
        b2PolygonShape bs2;
        bs2.SetAsBox(6,0.1, b2Vec2(a+0.0f,b+0.5f), 0);
        fd2->shape = &bs2;
        b2FixtureDef *fd3 = new b2FixtureDef;
        fd3->density = 10.0;
        fd3->friction = 0.5;
        fd3->restitution = 1.f;
        fd3->shape = new b2PolygonShape;
        b2PolygonShape bs3;
        bs3.SetAsBox(0.1,3.5, b2Vec2(a+6.0f,b+4.f), 0);
        fd3->shape = &bs3;
        b2FixtureDef *fd4 = new b2FixtureDef;
        fd4->density = 10.0;
        fd4->friction = 0.5;
        fd4->restitution = 1.f;
        fd4->shape = new b2PolygonShape;
        b2PolygonShape bs4;
        bs4.SetAsBox(0.1,1.75, b2Vec2(a-6.0f,b+5.75f), 0);
        fd4->shape = &bs4;
        b2FixtureDef *fd5 = new b2FixtureDef;
        fd5->density = 10.0;
        fd5->friction = 0.5;
        fd5->restitution = 1.f;
        fd5->shape = new b2PolygonShape;
        b2PolygonShape bs5;
        bs5.SetAsBox(0.1,0.5, b2Vec2(a-6.0f,b+1.f), 0);
        fd5->shape = &bs5;
        b2FixtureDef *fd6 = new b2FixtureDef;
        fd6->density = 10.0;
        fd6->friction = 0.5;
        fd6->restitution = 0.5f;
        fd6->shape = new b2PolygonShape;
        b2PolygonShape bs6;
        bs6.SetAsBox(3.0,0.1, b2Vec2(a-3.0f,b+1.5f), 0);
        fd6->shape = &bs6;

        b2Body* box1 = m_world->CreateBody(bd);
        box1->CreateFixture(fd1);
        box1->CreateFixture(fd2);
        box1->CreateFixture(fd3);
        box1->CreateFixture(fd4);
        box1->CreateFixture(fd5);
        box1->CreateFixture(fd6);
      }
      {
    //The sloping plank

        b2Body* sbody;
        b2PolygonShape poly;
        b2Vec2 vertices[4];
        vertices[0].Set(a-6.0f,b+6.6f);
        vertices[1].Set(a-6.0f,b+6.4f);
        vertices[2].Set(a-11.2f,b+3.2f);
        vertices[3].Set(a-11.2f,b+3.4f);
        poly.Set(vertices, 4);
        b2FixtureDef wedgefd;
        wedgefd.shape = &poly;
        wedgefd.density = 10.0f;
        wedgefd.friction = 0.0f;
        wedgefd.restitution = 0.0f;
        b2BodyDef wedgebd;
        wedgebd.position.Set(0.0f, 0.0f);
        sbody = m_world->CreateBody(&wedgebd);
        sbody->CreateFixture(&wedgefd);
      }

    //Snooker Ball
      {

        b2Body* spherebody;

        b2CircleShape circle;
        circle.m_radius = 0.5;

        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = 4.2f;
        ballfd.friction = 0.0f;
        ballfd.restitution = 1.0f;

        b2BodyDef ballbd;
        ballbd.type = b2_dynamicBody;
        ballbd.position.Set(a-5.8f,b+8.f);
        spherebody = m_world->CreateBody(&ballbd);
        spherebody->CreateFixture(&ballfd);

        b2Vec2 vel = spherebody->GetLinearVelocity();
        vel.x = 3.0f;
        vel.y = 2.0f;
        spherebody->SetLinearVelocity(vel);
        spherebody->SetGravityScale(0.0f);
      }

    //ball starting newton's cradle
      {

        b2Body* spherebody;

        b2CircleShape circle;
        circle.m_radius = 0.75f;

        b2FixtureDef ball;
        ball.shape = &circle;
        ball.density = 1.05f;
        ball.friction = 0.2f;
        ball.restitution = 0.5f;

        b2BodyDef ballBody;
        ballBody.type = b2_dynamicBody;
        ballBody.position.Set(a-2.0f,b+7.35f);
        spherebody = m_world->CreateBody(&ballBody);
        spherebody->CreateFixture(&ball);

      }
    }
    //Newton's Cradle
    {
      float a = 10.0f;
      float b = 68.0f;

    //horizontal shelf
      b2BodyDef *bd = new b2BodyDef;
      bd->position.Set(a-18,b+13);
      bd->fixedRotation = true;

      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(6,0.3, b2Vec2(0.f,0.f), 0);
      fd1->shape = &bs1;

      b2Body* b4 = m_world->CreateBody(bd);
      b4->CreateFixture(fd1);

    //pendullum

      b2Body* spherebody;
      b2CircleShape circle;
      circle.m_radius = 0.75f;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 1.f; 

      for(float i = 0.0; i<8.0f; i = i + 1.505f) {
        b2BodyDef bd1;
        bd1.type = b2_dynamicBody;
        bd1.position.Set(a-i-13.5f, b+5.0f);
        spherebody = m_world->CreateBody(&bd1);
        spherebody->CreateFixture(&ballfd);

        b2RevoluteJointDef jd;
        b2Vec2 anchor;
        anchor.Set(a-i-13.5f, b+13.0f);
        jd.Initialize(b4, spherebody, anchor);
        m_world->CreateJoint(&jd);
      } 
    }

    //system hit after cradle
    {
      float a = 10.0f;
      float b = 70.0f;
      {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.0f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 20.0f;
        fd.friction = 0.1f;

        for (int i = 0; i < 2; ++i)
        {
          b2BodyDef bd;
          bd.type = b2_dynamicBody;
          bd.position.Set(a-28.41f - 1.5f * i, b+4.75f);
          b2Body* body = m_world->CreateBody(&bd);
          body->CreateFixture(&fd);
        }
      }
      //shelf
      {
        b2BodyDef *bd = new b2BodyDef;
        bd->position.Set(a-30.5f,b + 4.75f);
        bd->fixedRotation = true;

        b2FixtureDef *fd1 = new b2FixtureDef;
        fd1->density = 10.0;
        fd1->friction = 1.;
        fd1->shape = new b2PolygonShape;
        b2PolygonShape bs1;
        bs1.SetAsBox(3.1,0.1, b2Vec2(0.f,0.f), 0);
        fd1->shape = &bs1;

        b2Body* b4 = m_world->CreateBody(bd);
        b4->CreateFixture(fd1);
      }     

      //1st Ball
      {
        b2Body* spherebody;

        b2CircleShape circle;
        circle.m_radius = 0.75f;

        b2FixtureDef ball;
        ball.shape = &circle;
        ball.density = 1.0f;
        ball.friction = 0.2f;
        ball.restitution = 0.35f;

        b2BodyDef ballBody;
        ballBody.type = b2_dynamicBody;
        ballBody.position.Set(a-32.5f,b+5.1f);
        spherebody = m_world->CreateBody(&ballBody);
        spherebody->CreateFixture(&ball);        
      }

      //vertical hinged plank
      {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 2.5f);

        b2BodyDef bd;
        bd.position.Set(a-34.5f, b+7.75f);
        bd.type = b2_dynamicBody;
        b2Body* body = m_world->CreateBody(&bd);
        b2FixtureDef *fd = new b2FixtureDef;
        fd->density = 3.f;
        fd->shape = new b2PolygonShape;
        fd->shape = &shape;
        body->CreateFixture(fd);

        b2PolygonShape shape2;
        shape2.SetAsBox(0.1f, 2.5f);
        b2BodyDef bd2;
        bd2.position.Set(a-34.5, b+7.5f);
        b2Body* body2 = m_world->CreateBody(&bd2);

        b2RevoluteJointDef jointDef;
        jointDef.bodyA = body;
        jointDef.bodyB = body2;
        jointDef.localAnchorA.Set(0,0);
        jointDef.localAnchorB.Set(0,0);
        jointDef.collideConnected = false;
        m_world->CreateJoint(&jointDef);        
      }

      //horizontal plan on top
      {
        b2BodyDef *bd = new b2BodyDef;
        bd->position.Set(a-32.5f,b+8.5f);
        bd->fixedRotation = true;

        b2FixtureDef *fd1 = new b2FixtureDef;
        fd1->density = 10.0;
        fd1->friction = 0.5;
        fd1->shape = new b2PolygonShape;
        b2PolygonShape bs1;
        bs1.SetAsBox(1,0.1, b2Vec2(0.f,0.f), 0);
        fd1->shape = &bs1;

        b2Body* b4 = m_world->CreateBody(bd);
        b4->CreateFixture(fd1);        
      }

      //2nd Ball
      {
        b2Body* spherebody;

        b2CircleShape circle;
        circle.m_radius = 0.75f;

        b2FixtureDef ball;
        ball.shape = &circle;
        ball.density = 2.3f;
        ball.friction = 0.0f;
        ball.restitution = 0.5f;

        b2BodyDef ballBody;
        ballBody.type = b2_dynamicBody;
        ballBody.position.Set(a-32.75f,b+9.25f);
        spherebody = m_world->CreateBody(&ballBody);
        spherebody->CreateFixture(&ball);         
      }

      //Fixed Vertical shelf (Support)
      {
        b2BodyDef *bd = new b2BodyDef;
        bd->position.Set(a-29.5f,b+9.f);
        bd->fixedRotation = true;

        b2FixtureDef *fd1 = new b2FixtureDef;
        fd1->density = 10.0;
        fd1->friction = 0.5;
        fd1->restitution = 1.0;
        fd1->shape = new b2PolygonShape;
        b2PolygonShape bs1;
        bs1.SetAsBox(0.1,1.25, b2Vec2(0.f,0.f), 0);
        fd1->shape = &bs1;

        b2Body* b4 = m_world->CreateBody(bd);
        b4->CreateFixture(fd1);         
      }

    }
    //The see-saw system at the bottom
    {
      float coordinatex=30.5f;
      float coordinatey=31.5f;
      {
        b2PolygonShape plank;
        plank.SetAsBox(0.25f, 20.0f);
        b2BodyDef bd;
        bd.position.Set(coordinatex-4, coordinatey+12.5);
        bd.angle =  b2_pi*0.5;
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&plank, 0.0f);
      }

      {
      //The triangle wedge
        b2Body* sbody;
        b2PolygonShape poly;
        b2Vec2 vertices[3];
        vertices[0].Set(-2,0);
        vertices[1].Set(2,0);
        vertices[2].Set(0,4);
        poly.Set(vertices, 3);
        b2FixtureDef wedgefd;
        wedgefd.shape = &poly;
        wedgefd.density = 10.0f;
        wedgefd.friction = 0.0f;
        wedgefd.restitution = 0.0f;
        b2BodyDef wedgebd;
        wedgebd.position.Set(coordinatex-2, coordinatey+14.1-1.5);
        sbody = m_world->CreateBody(&wedgebd);
        sbody->CreateFixture(&wedgefd);

      //The plank on top of the wedge
        b2PolygonShape shape;
        shape.SetAsBox(15.0f, 0.2f);
        b2BodyDef bd2;
        bd2.position.Set(coordinatex-1.5f, coordinatey+14.1f+2.5);
        bd2.type = b2_dynamicBody;
        b2Body* body = m_world->CreateBody(&bd2);
        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = 1.f;
        fd2->shape = new b2PolygonShape;
        fd2->shape = &shape;
        body->CreateFixture(fd2);

      //The joint between the wedge and plank
        b2RevoluteJointDef jd;
        b2Vec2 anchor;
        anchor.Set(coordinatex-2, coordinatey+14.1f+2.5);
        jd.Initialize(sbody, body, anchor);
        m_world->CreateJoint(&jd);

      //The open box
        b2BodyDef *bd1 = new b2BodyDef;
        bd1->type = b2_dynamicBody;
        bd1->position.Set(coordinatex+5.0f,coordinatey+30.0f);
        bd1->fixedRotation = true;
        b2FixtureDef *fd1 = new b2FixtureDef;
        fd1->density = 17.0;
        fd1->friction = 1.0;
        fd1->restitution = 0.f;
        fd1->shape = new b2PolygonShape;
        b2PolygonShape bs1;
        bs1.SetAsBox(0.2,2, b2Vec2(0.f,-1.9f), 0);
        fd1->shape = &bs1;
        b2Body* box1 = m_world->CreateBody(bd1);
        box1->CreateFixture(fd1);

      //barrier
        b2PolygonShape plank;
        plank.SetAsBox(0.25f, 2.0f);
        b2BodyDef bd;
        bd.type=b2_dynamicBody;
        bd.position.Set(coordinatex-10.5f, coordinatey+18.0f);
        bd.angle =  b2_pi;
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&plank, 0.0f);

      // b2RevoluteJointDef Joint14;
      // b2Vec2 anchor14;
      // anchor14.Set(a+25.6f, b+12.f);
      // Joint14.collideConnected = true; 
      // Joint14.Initialize(sbody1, sbody4, anchor14);
      // m_world->CreateJoint(&Joint14);


        b2RevoluteJointDef Joint14;
        b2Vec2 anchor14;
        anchor14.Set(coordinatex-10.0f, coordinatey+18.0f);
        Joint14.collideConnected=false;
        Joint14.Initialize(ground,body,anchor14);
        m_world->CreateJoint(&Joint14);

        b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(coordinatex-1.5-10.0, coordinatey+14.1f+2.5); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(coordinatex+5.0,coordinatey+30.0f); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(coordinatex-1.5-10.0, coordinatey+14.1f+7.5+5+5); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(coordinatex+5.0f,coordinatey+30+5); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(body, box1, worldAnchorGround1, worldAnchorGround2, worldAnchorOnBody1, worldAnchorOnBody2, ratio);
      m_world->CreateJoint(myjoint);

      
    }
  }


  {
    {
      float coordinatey = 60.0f;
      float coordinatex = -40.0f;
    //pulley
      {
        float i=coordinatex+17;
        float j=coordinatey-15;
        b2BodyDef *bd = new b2BodyDef;
        bd->type = b2_dynamicBody;
        bd->position.Set(i,j);
        bd->fixedRotation = true;

      //The open box
        b2FixtureDef *fd1 = new b2FixtureDef;
        fd1->density = 10.0;
        fd1->friction = 1.0;
        fd1->restitution = 0.f;
        fd1->shape = new b2PolygonShape;
        b2PolygonShape bs1;
        bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
        fd1->shape = &bs1;
        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = 10.0;
        fd2->friction = 1.0;
        fd2->restitution = 0.f;
        fd2->shape = new b2PolygonShape;
        b2PolygonShape bs2;
        bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
        fd2->shape = &bs2;
        b2FixtureDef *fd3 = new b2FixtureDef;
        fd3->density = 10.0;
        fd3->friction = 1.0;
        fd3->restitution = 0.f;
        fd3->shape = new b2PolygonShape;
        b2PolygonShape bs3;
        bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
        fd3->shape = &bs3;

        b2Body* box1 = m_world->CreateBody(bd);
        box1->CreateFixture(fd1);
        box1->CreateFixture(fd2);
        box1->CreateFixture(fd3);
      //The bar
        float i1=coordinatex+5;
        float j1=coordinatey-15.0f;
        bd->position.Set(i1,j1);  
        fd1->density = 31.5;    
        b2Body* box2 = m_world->CreateBody(bd);
        box2->CreateFixture(fd1);

      // The pulley joint
        b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(i, j); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(i1, j1); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(i, j+5); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(i1, j1+5); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }
    
    // horizontal plank on extreme left
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
      float i=coordinatex+0.8;
      float j=coordinatey-12.7f;
      b2BodyDef bd;
      bd.position.Set(i, j);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(i, j+2);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }

    // float coordinatey = 30.0f;
    // float coordinatex = -40.0f;
    {

      float width=4.9f;
      float heightdiff=20.0/4;
      float millx=coordinatex+13.0+6.0/2;
      float milly=coordinatey-6;    
      float firstmillx=milly+width/2;

      b2PolygonShape shape1;
      shape1.SetAsBox(width/2,0.10);
      b2PolygonShape shape2;
      shape2.SetAsBox(0.10,width/2);

      b2FixtureDef millfd;
      millfd.shape = &shape1;
      millfd.density = 1000000.0f;
      millfd.friction = 0.0f;
      millfd.restitution = 0.0f;

      b2FixtureDef millfd2;
      millfd2.shape = &shape2;
      millfd2.density = 1000000.0f;
      millfd2.friction = 0.0f;
      millfd2.restitution = 0.0f;


      b2Body* b2;
      b2PolygonShape shape;
      shape.SetAsBox(0.0f, 0.0f);      

      for(int i=0;i<4;i++){

        b2BodyDef bx;
        bx.position.Set(millx,firstmillx+i*heightdiff);
        b2 = m_world->CreateBody(&bx);
        b2->CreateFixture(&shape, 10.0f);

        b2BodyDef bd;
        bd.position.Set(millx,firstmillx+i*heightdiff);
        bd.type=b2_dynamicBody;
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&millfd);

        b2BodyDef bd1;
        bd1.position.Set(millx,firstmillx+i*heightdiff);
        bd1.type=b2_dynamicBody;
        b2Body* ground1 = m_world->CreateBody(&bd1);
        ground1->CreateFixture(&millfd2);
        ground1->SetAngularVelocity(-3.14/2);

        b2WeldJointDef  joint_def;
        joint_def.bodyA = ground;
        joint_def.bodyB = ground1;

      //add the joint to the world
        m_world->CreateJoint(&joint_def);

        b2RevoluteJointDef jd;
        b2Vec2 anchor;
        anchor.Set(millx,firstmillx+i*heightdiff);
        jd.Initialize(b2,ground, anchor);
        m_world->CreateJoint(&jd);  

    //Vertical Wall on Right and Left
        {
          float millx=coordinatex+13.0;
          float width=6.0f;
          float height=20.0f;
          float milly=coordinatey+height/2-3;
          b2PolygonShape shape;
          shape.SetAsBox(0.25,height/2);

          b2BodyDef bd;
          bd.position.Set(millx,milly-4);
          b2Body* ground = m_world->CreateBody(&bd);
          ground->CreateFixture(&shape, 0.0f);

          bd.position.Set(millx+width,milly-4);
          ground = m_world->CreateBody(&bd);
          ground->CreateFixture(&shape, 0.0f);


        }    } 
      } 
    }
  //Three end jointed rods
    {
      float coordinatex=5.0f;
      float coordinatey=50.0f;

  //first vertical
      {

        b2PolygonShape shape;
        shape.SetAsBox(0.2f, 2.2f);
        float i=coordinatex;
        float j=coordinatey+7.6;
        b2BodyDef bd;
        bd.position.Set(i, j);
        bd.type = b2_dynamicBody;
        b2Body* body = m_world->CreateBody(&bd);
        b2FixtureDef *fd = new b2FixtureDef;
        fd->density = 1.f;
        fd->shape = new b2PolygonShape;
        fd->shape = &shape;
        body->CreateFixture(fd);

        b2PolygonShape shape2;
        shape2.SetAsBox(0.2f, 2.0f);
        b2BodyDef bd2;
        bd2.position.Set(i, j+2);
        b2Body* body2 = m_world->CreateBody(&bd2);

        b2RevoluteJointDef jointDef;
        jointDef.bodyA = body;
        jointDef.bodyB = body2;
        jointDef.localAnchorA.Set(0,-2.2);
        jointDef.localAnchorB.Set(0,-2.2);
        jointDef.collideConnected = false;
        m_world->CreateJoint(&jointDef);
      }
    //second one
      {

        b2PolygonShape shape;
        shape.SetAsBox(0.2f, 2.2f);
        float i=coordinatex;
        float j=coordinatey+7.6+7.6;
        b2BodyDef bd;
        bd.position.Set(i, j);
        bd.type = b2_dynamicBody;
        b2Body* body = m_world->CreateBody(&bd);
        b2FixtureDef *fd = new b2FixtureDef;
        fd->density = 1.f;
        fd->shape = new b2PolygonShape;
        fd->shape = &shape;
        body->CreateFixture(fd);

        b2PolygonShape shape2;
        shape2.SetAsBox(0.2f, 2.0f);
        b2BodyDef bd2;
        bd2.position.Set(i, j+2);
        b2Body* body2 = m_world->CreateBody(&bd2);

        b2RevoluteJointDef jointDef;
        jointDef.bodyA = body;
        jointDef.bodyB = body2;
        jointDef.localAnchorA.Set(0,-2.2);
        jointDef.localAnchorB.Set(0,-2.2);
        jointDef.collideConnected = false;
        m_world->CreateJoint(&jointDef);
      }
    //the horizontal plank for ball
      {
        b2PolygonShape plank;
        plank.SetAsBox(0.25f, 4.0f);
        b2BodyDef bd;
        bd.position.Set(coordinatex-4.5, coordinatey+16);
        bd.angle =  0.5f*b2_pi;
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&plank, 0.0f);
      }
    //slant one for ball
      {
        b2PolygonShape plank;
        plank.SetAsBox(0.25f, 2.0f);
        b2BodyDef bd;
        bd.position.Set(coordinatex-8.8, coordinatey+17.5);
        bd.angle =  b2_pi*0.2f;
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&plank, 0.0f);
      }
    //container for ball
      {
        b2BodyDef *bd = new b2BodyDef;
        bd->type = b2_staticBody;
        bd->position.Set(coordinatex+4.0,coordinatey+9.5);
        bd->fixedRotation = true;
        b2FixtureDef *fd1 = new b2FixtureDef;
        fd1->density = 10.0;
        fd1->friction = 0.5;
        fd1->restitution = 0.f;
        fd1->shape = new b2PolygonShape;
        b2PolygonShape bs1;
        bs1.SetAsBox(1.5,0.2, b2Vec2(0.f,-1.4f), 0);
        fd1->shape = &bs1;
        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = 10.0;
        fd2->friction = 0.5;
        fd2->restitution = 0.f;
        fd2->shape = new b2PolygonShape;
        b2PolygonShape bs2;
        bs2.SetAsBox(0.2,1.5, b2Vec2(1.5f,0.f), 0);
        fd2->shape = &bs2;
        b2FixtureDef *fd3 = new b2FixtureDef;
        fd3->density = 10.0;
        fd3->friction = 0.5;
        fd3->restitution = 0.f;
        fd3->shape = new b2PolygonShape;
        b2PolygonShape bs3;
        bs3.SetAsBox(0.2,1.5, b2Vec2(-1.5f,0.f), 0);
        fd3->shape = &bs3;

        b2Body* box1 = m_world->CreateBody(bd);
        box1->CreateFixture(fd1);
      // box1->CreateFixture(fd2);
      // box1->CreateFixture(fd3);
      }
    //for the ball to go along the slant one
      {
        b2PolygonShape plank;
        plank.SetAsBox(0.25f, 12.2f);
        b2BodyDef bd;
        bd.position.Set(coordinatex+18.0f, coordinatey+7.0f);
        bd.angle =  0.48*b2_pi;
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&plank, 0.0f);
      }
    //Plank for dominos1
      {
        b2PolygonShape plank;
        plank.SetAsBox(0.25f, 9.0f);
        b2BodyDef bd;
        bd.position.Set(coordinatex-12.5f, coordinatey+11.7f);
        bd.angle =  0.5*b2_pi;
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&plank, 0.0f);
      }
    //Dominos are here and the required ball
      {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.0f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 1.0f;
        fd.friction = 0.5f;

        for (int i = 0; i < 15; ++i)
        {
         b2BodyDef bd;
         bd.type = b2_dynamicBody;
         bd.position.Set(coordinatex-17.8 + 1.0f * i, coordinatey+12.7f);
         b2Body* body = m_world->CreateBody(&bd);
         body->CreateFixture(&fd);
       }   
       b2Body* spherebody;

       b2CircleShape circle;
       circle.m_radius = 0.5;

       b2FixtureDef ballfd;
       ballfd.shape = &circle;
       ballfd.density = 0.3f;
       ballfd.friction = 0.0f;
       ballfd.restitution = 0.5f;

       b2BodyDef ballbd;
       ballbd.type = b2_dynamicBody;
       ballbd.position.Set(coordinatex-18.8,coordinatey+12.5);
       spherebody = m_world->CreateBody(&ballbd);
       spherebody->CreateFixture(&ballfd);
     }
    //vertical plank between dominos 1& 2
     {
      b2PolygonShape plank;
      plank.SetAsBox(0.25f, 3.0f);
      b2BodyDef bd;
      bd.position.Set(coordinatex-24.0f, coordinatey+10.0f);
      bd.angle = b2_pi;
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&plank, 0.0f);
    }
    //Plank for dominos 2
    {
      b2PolygonShape plank;
      plank.SetAsBox(0.25f, 7.0f);
      b2BodyDef bd;
      bd.position.Set(coordinatex-14.5f, coordinatey+7.7f);
      bd.angle =  0.5*b2_pi;
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&plank, 0.0f);
    }
    //Dominos set 2 are here and the ball
    {
     b2PolygonShape shape;
     shape.SetAsBox(0.1f, 1.0f);

     b2FixtureDef fd;
     fd.shape = &shape;
     fd.density = 1.0f;
     fd.friction = 0.5f;

     for (int i = 1; i < 10; ++i)
     {
       b2BodyDef bd;
       bd.type = b2_dynamicBody;
       bd.position.Set(coordinatex-19.8 + 1.0f * i, coordinatey+8.7f);
       b2Body* body = m_world->CreateBody(&bd);
       body->CreateFixture(&fd);
     }   
     b2Body* spherebody;

     b2CircleShape circle;
     circle.m_radius = 0.5;

     b2FixtureDef ballfd;
     ballfd.shape = &circle;
     ballfd.density = 0.3f;
     ballfd.friction = 0.0f;
     ballfd.restitution = 0.5f;

     b2BodyDef ballbd;
     ballbd.type = b2_dynamicBody;
     ballbd.position.Set(coordinatex-8.8,coordinatey+8.5);
     spherebody = m_world->CreateBody(&ballbd);
     spherebody->CreateFixture(&ballfd);
   }
   
    //Vertical plank between dominos 2 and 3
   {
    b2PolygonShape plank;
    plank.SetAsBox(0.25f, 2.0f);
    b2BodyDef bd;
    bd.position.Set(coordinatex-5.5f, coordinatey+5.5f);
    bd.angle = b2_pi;
    b2Body* ground = m_world->CreateBody(&bd);
    ground->CreateFixture(&plank, 0.0f);
  }
    //Plank for dominos 3
  {
    b2PolygonShape plank;
    plank.SetAsBox(0.25f, 7.0f);
    b2BodyDef bd;
    bd.position.Set(coordinatex-13.5f, coordinatey+5.0f);
    bd.angle =  0.5*b2_pi;
    b2Body* ground = m_world->CreateBody(&bd);
    ground->CreateFixture(&plank, 0.0f);
  }
     //Dominos set 3 are here and the ball
  {
   b2PolygonShape shape;
   shape.SetAsBox(0.1f, 1.0f);

   b2FixtureDef fd;
   fd.shape = &shape;
   fd.density = 1.0f;
   fd.friction = 0.5f;

   for (int i = 1; i < 10; ++i)
   {
     b2BodyDef bd;
     bd.type = b2_dynamicBody;
     bd.position.Set(coordinatex-18.8 + 1.0f * i, coordinatey+6.0f);
     b2Body* body = m_world->CreateBody(&bd);
     body->CreateFixture(&fd);
   }   
   b2Body* spherebody;

   b2CircleShape circle;
   circle.m_radius = 0.5;

   b2FixtureDef ballfd;
   ballfd.shape = &circle;
   ballfd.density = 0.3f;
   ballfd.friction = 0.0f;
   ballfd.restitution = 0.5f;

   b2BodyDef ballbd;
   ballbd.type = b2_dynamicBody;
   ballbd.position.Set(coordinatex-18.8,coordinatey+5.3);
   spherebody = m_world->CreateBody(&ballbd);
   spherebody->CreateFixture(&ballfd);
 }
    //vertical plank between dominos 3 & 4
 {
  b2PolygonShape plank;
  plank.SetAsBox(0.25f, 3.0f);
  b2BodyDef bd;
  bd.position.Set(coordinatex-22.9f, coordinatey+5.0f);
  bd.angle = b2_pi;
  b2Body* ground = m_world->CreateBody(&bd);
  ground->CreateFixture(&plank, 0.0f);
}
    //Plank for dominos4
{
  b2PolygonShape plank;
  plank.SetAsBox(0.25f, 13.0f);
  b2BodyDef bd;
  bd.position.Set(coordinatex-10.5f, 52);
  bd.angle =  0.5f*b2_pi;
  b2Body* ground = m_world->CreateBody(&bd);
  ground->CreateFixture(&plank, 0.0f);
} 
    //Dominos set 4 are here and the ball
{
 b2PolygonShape shape;
 shape.SetAsBox(0.1f, 1.0f);

 b2FixtureDef fd;
 fd.shape = &shape;
 fd.density = 1.0f;
 fd.friction = 0.5f;

 for (int i = 1; i < 10; ++i)
 {
   b2BodyDef bd;
   bd.type = b2_dynamicBody;
   bd.position.Set(coordinatex-19.8 + 1.0f * i, 53);
   b2Body* body = m_world->CreateBody(&bd);
   body->CreateFixture(&fd);
 }   
 b2Body* spherebody;

 b2CircleShape circle;
 circle.m_radius = 0.5;

 b2FixtureDef ballfd;
 ballfd.shape = &circle;
 ballfd.density = 0.5f;
 ballfd.friction = 0.0f;
 ballfd.restitution = 0.5f;

 b2BodyDef ballbd;
 ballbd.type = b2_dynamicBody;
 ballbd.position.Set(coordinatex-8.8,52.9);
 spherebody = m_world->CreateBody(&ballbd);
 spherebody->CreateFixture(&ballfd);
}

}
{
  float coordinatex=5.0f;
  float coordinatey=52.0f;
   //link to next
  {
    b2PolygonShape plank;
    plank.SetAsBox(0.25f, 4.0f);
    b2BodyDef bd;
    bd.position.Set(coordinatex+5, coordinatey);
    bd.angle =  0.5f*b2_pi;
    b2Body* ground = m_world->CreateBody(&bd);
    ground->CreateFixture(&plank, 0.0f);
  }  
  {
    b2Body* spherebody;

    b2CircleShape circle;
    circle.m_radius = 0.5;

    b2FixtureDef ballfd;
    ballfd.shape = &circle;
    ballfd.density = 34.0f;
    ballfd.friction = 0.5f;
    ballfd.restitution = 1.0f;

    b2BodyDef ballbd;
    ballbd.type = b2_dynamicBody;
    ballbd.position.Set(coordinatex+1,coordinatey+0.8);
    spherebody = m_world->CreateBody(&ballbd);
    spherebody->CreateFixture(&ballfd);
  }
}




  // }//Container for the fluid.
{
  float i = 66.0f;
  float j = 10.0f;
    //Vertical wall on left
  {
    b2Vec2 vertices[4];
    vertices[0].Set(0.0f, 0.0f);
    vertices[1].Set(0.0f, -18.0f);
    vertices[2].Set(-0.5f, -18.0f);
    vertices[3].Set(-0.5,0);
      // vertices[4].Set(0.5,-3);
      // vertices[5].Set(0.5,-2);
      // vertices[6].Set(-0.5,-1);
      // vertices[7].Set(-0.5,0);
    int32 count = 4;



    b2PolygonShape polygonShape;
    b2FixtureDef myFixtureDef;
    b2BodyDef myBodyDef;
      myBodyDef.type = b2_staticBody; //this will be a dynamic body
      myBodyDef.position.Set(i+5, j+45); //a little to the left

    // b2Body* dynamicBody1 = m_world->CreateBody(&myBodyDef);
      polygonShape.Set(vertices, count);
      myFixtureDef.shape = &polygonShape; //change the shape of the fixture
      myBodyDef.position.Set(i-28.2, j+33.3f); //in the middle
      b2Body* dynamicBody2 = m_world->CreateBody(&myBodyDef);
      dynamicBody2->CreateFixture(&myFixtureDef); //add a fixture to the body
      vertices[0].Set(0.0f, 0.0f);
      vertices[1].Set(0.5f, 0.5f);
      vertices[2].Set(20.0f, 20.0f);
      vertices[3].Set(20.5,20.5);
     //  count=4;
     //   polygonShape.Set(vertices, count);
     //  myFixtureDef.shape = &polygonShape; //change the shape of the fixture
     //  myBodyDef.position.Set(-20, 20); //in the middle
     // dynamicBody2 = m_world->CreateBody(&myBodyDef);
     //  dynamicBody2->CreateFixture(&myFixtureDef); //add a fixtur
    }
    //Vertical Wall on Right
    {
      b2Vec2 vertices[4];
      vertices[0].Set(0.0f, 0.0f);
      vertices[1].Set(0.0f, -20.0f);
      vertices[2].Set(-0.5f, -20.0f);
      vertices[3].Set(-0.5,0);
      // vertices[4].Set(0.5,-3);
      // vertices[5].Set(0.5,-2);
      // vertices[6].Set(-0.5,-1);
      // vertices[7].Set(-0.5,0);
      int32 count = 4;



      b2PolygonShape polygonShape;
      b2FixtureDef myFixtureDef;
      b2BodyDef myBodyDef;
      myBodyDef.type = b2_staticBody; //this will be a dynamic body
      myBodyDef.position.Set(i+5, j+50); //a little to the left

    // b2Body* dynamicBody1 = m_world->CreateBody(&myBodyDef);
      polygonShape.Set(vertices, count);
      myFixtureDef.shape = &polygonShape; //change the shape of the fixture
      myBodyDef.position.Set(i+8.7f,j+35.0f); //in the middle
      b2Body* dynamicBody2 = m_world->CreateBody(&myBodyDef);
      dynamicBody2->CreateFixture(&myFixtureDef); //add a fixture to the body
      vertices[0].Set(0.0f, 0.0f);
      vertices[1].Set(0.5f, 0.5f);
      vertices[2].Set(20.0f, 20.0f);
      vertices[3].Set(20.5,20.5);
     //  count=4;
     //   polygonShape.Set(vertices, count);
     //  myFixtureDef.shape = &polygonShape; //change the shape of the fixture
     //  myBodyDef.position.Set(-20, 20); //in the middle
     // dynamicBody2 = m_world->CreateBody(&myBodyDef);
     //  dynamicBody2->CreateFixture(&myFixtureDef); //add a fixtur
    }
    // Slanted plank on left
    {
      b2PolygonShape plank;
      plank.SetAsBox(0.25f, 10.0f);
      b2BodyDef bd;
      bd.position.Set(i-20.0f, j+12.0f);
      bd.angle = 0.4f * b2_pi;
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&plank, 0.0f);
    }
    //Slanted Plank to right
    {
      b2PolygonShape plank;
      plank.SetAsBox(0.25f, 10.0f);
      b2BodyDef bd;
      bd.position.Set(i,j+12.0f);
      bd.angle = -0.4f * b2_pi;
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&plank, 0.0f);
    }
    //Vertical plank on left
    {
      b2PolygonShape plank;
      plank.SetAsBox(0.25f, 2.0f);
      b2BodyDef bd;
      bd.position.Set(i-11.0f, j+7.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&plank, 0.0f);
    }
     //Vertical plank on right
    {
      b2PolygonShape plank;
      plank.SetAsBox(0.25f, 2.0f);
      b2BodyDef bd;
      bd.position.Set(i-9.0f, j+7.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&plank, 0.0f);
    }



    //Fluid
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_staticBody;
      bd->position.Set(i-2,j+16);
      bd->fixedRotation = true;
      
      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;

      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      //The bar
      // bd->position.Set(10,15);  
      // fd1->density = 34.0;    
      // b2Body* box2 = m_world->CreateBody(bd);
      // box2->CreateFixture(fd1);

      // The pulley joint
      // b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      // b2Vec2 worldAnchorOnBody1(-10, 15); // Anchor point on body 1 in world axis
      // b2Vec2 worldAnchorOnBody2(10, 15); // Anchor point on body 2 in world axis
      // b2Vec2 worldAnchorGround1(-10, 20); // Anchor point for ground 1 in world axis
      // b2Vec2 worldAnchorGround2(10, 20); // Anchor point for ground 2 in world axis
      // float32 ratio = 1.0f; // Define ratio
      // myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      // m_world->CreateJoint(myjoint);
    }
    //Another container
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_staticBody;
      bd->position.Set(i-19,j+16);
      bd->fixedRotation = true;
      
      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;

      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      //The bar
      // bd->position.Set(10,15);  
      // fd1->density = 34.0;    
      // b2Body* box2 = m_world->CreateBody(bd);
      // box2->CreateFixture(fd1);
    }

    {
      b2Body* spherebody;

      b2CircleShape circle;
      circle.m_radius = 0.1;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 0.005f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      for(int n=0;n<24;n++) {
        for (int m = 1; m < 18; ++m) {
          b2BodyDef ballbd;
          ballbd.type = b2_dynamicBody;
          ballbd.position.Set(i-3.8f + m*0.2, j+14.8f+n*0.5);
          spherebody = m_world->CreateBody(&ballbd);
          spherebody->CreateFixture(&ballfd);
        }
      }
    }
    {
      b2Body* spherebody;

      b2CircleShape circle;
      circle.m_radius = 0.1;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 0.005f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      for(int n=0;n<24;n++) {
        for (int m = 1; m < 18; ++m) {
          b2BodyDef ballbd;
          ballbd.type = b2_dynamicBody;
          ballbd.position.Set(i-20.8f + m*0.2f, j+14.8f+n*0.5);
          spherebody = m_world->CreateBody(&ballbd);
          spherebody->CreateFixture(&ballfd);
        }
      }
    }
  //Adding two balls for falling the plank
    {
      b2PolygonShape shape;
      shape.SetAsBox(3.2f, 0.2f);
      float i1=i-11;
      float j1=j+28;
      b2BodyDef bd;
      bd.position.Set(i1, j1);
      // bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->friction = 0.0f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      // b2PolygonShape shape2;
      // shape2.SetAsBox(0.2f, 2.0f);
      // b2BodyDef bd2;
      // bd2.position.Set(i1, j1+2);
      // b2Body* body2 = m_world->CreateBody(&bd2);
    }
    //Two balls for falling
    {
      b2Body* spherebody;

      b2CircleShape circle;
      circle.m_radius = 0.75f;

      b2FixtureDef ball;
      ball.shape = &circle;
      ball.density = 50.0f;
      ball.friction = 0.2f;
      ball.restitution = 0.5f;

      b2BodyDef ballBody;
      ballBody.type = b2_dynamicBody;
      ballBody.position.Set(i-11-1.0f,j+28+1.0f);
      spherebody = m_world->CreateBody(&ballBody);
      spherebody->CreateFixture(&ball);
      {
        b2Body* spherebody;

        b2CircleShape circle;
        circle.m_radius = 0.75f;

        b2FixtureDef ball;
        ball.shape = &circle;
        ball.density = 50.0f;
        ball.friction = 0.2f;
        ball.restitution = 0.5f;

        b2BodyDef ballBody;
        ballBody.type = b2_dynamicBody;
        ballBody.position.Set(i-11+1.0f,j+28+1.0f);
        spherebody = m_world->CreateBody(&ballBody);
        spherebody->CreateFixture(&ball);
      }
    }
    //The ball hitting the two balls
    {
      b2Body* spherebody;

      b2CircleShape circle;
      circle.m_radius = 0.75f;

      b2FixtureDef ball;
      ball.shape = &circle;
      ball.density = 9.0f;
      ball.friction = 0.2f;
      ball.restitution = 0.5f;

      b2BodyDef ballBody;
      ballBody.type = b2_dynamicBody;
      ballBody.position.Set(i-14.5f,j+45+1.0f);
      spherebody = m_world->CreateBody(&ballBody);
      spherebody->CreateFixture(&ball);
    }
  //Fluid ends here.

  }
  //Container code ends.
    //*//*//

}
    //For the big ball to go
{
  {
    float i=66.0f;
    float j=10.0f;
    b2PolygonShape shape;
    shape.SetAsBox(10.0f, 0.2f);
      // float i=coordinatex+0.8;
      // float j=coordinatey-12.7f;
    b2BodyDef bd;
    bd.position.Set(i-24.0f, j+44.0f);
      // bd.type = b2_dynamicBody;
    b2Body* body = m_world->CreateBody(&bd);
    b2FixtureDef *fd = new b2FixtureDef;
    fd->density = 1.f;
    fd->friction = 0.0f;
    fd->shape = new b2PolygonShape;
    fd->shape = &shape;
    body->CreateFixture(fd);
  }
}
    //The platform
{
  float i=66.0f;
  float j=10.0f;
  b2PolygonShape shape;
  shape.SetAsBox(5.0f, 0.2f);

  b2BodyDef bd;
  bd.position.Set(i-10.0f, j-7.0f);
      // bd.type = b2_dynamicBody;
  b2Body* body = m_world->CreateBody(&bd);
  b2FixtureDef *fd = new b2FixtureDef;
  fd->density = 1.f;
  fd->friction = 1.5f;
  fd->shape = new b2PolygonShape;
  fd->shape = &shape;
  body->CreateFixture(fd);

}
   //Container for the high density ball
{
  {
    float i=66.0f;
    float j=10.0f;
    b2BodyDef *bd = new b2BodyDef;
    bd->type = b2_staticBody;
    bd->position.Set(i-62,j+32);
    bd->fixedRotation = true;

      //The open box
    b2FixtureDef *fd1 = new b2FixtureDef;
    fd1->density = 10.0;
    fd1->friction = 0.5;
    fd1->restitution = 0.f;
    fd1->shape = new b2PolygonShape;
    b2PolygonShape bs1;
    bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
    fd1->shape = &bs1;
    b2FixtureDef *fd2 = new b2FixtureDef;
    fd2->density = 10.0;
    fd2->friction = 0.5;
    fd2->restitution = 0.f;
    fd2->shape = new b2PolygonShape;
    b2PolygonShape bs2;
    bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
    fd2->shape = &bs2;
    b2FixtureDef *fd3 = new b2FixtureDef;
    fd3->density = 10.0;
    fd3->friction = 56.0f;
    fd3->restitution = 0.f;
    fd3->shape = new b2PolygonShape;
    b2PolygonShape bs3;
    bs3.SetAsBox(0.2,3, b2Vec2(-2.0f,1.f), 0);
    fd3->shape = &bs3;

    b2Body* box1 = m_world->CreateBody(bd);
    box1->CreateFixture(fd1);
    box1->CreateFixture(fd2);
    box1->CreateFixture(fd3);
  }
}


  //The conveyor belt
{
      //overall shift parameters for the entire conveyor belt system
  double conveyorleft = -30.0;
  double conveyorup = 0.5;
       //The number of links in the conveyor belt system either above or below.
       //The total number of links will be 2*links + 4 (2 links also on either side)
       int links = 40; //Divisible by 20
       
       double speed = -5*PI/3; //This is the speed at which the conveyor belts' driving wheels rotate
       double frction = 10.0f;
       
       //The links are made as follows.
       //First, we make the lower chain.
       //Then, we make the upper chain
       //Then, we make the two links which join these two chains on the left hand side
       //Then, we make the two links which join these two chains on the right hand side 
       b2Body* leftup; //this is the body which is the top left link
       b2Body* rightup; //this is the body which is the top right link
       b2Body* leftdown; //this is the body which is the bottom left link
       b2Body* rightdown; //this is the body which is the bottom right link
       //Note that these four special links are required because they are used specifically to connect the upper and lower chains
       
       //here we initialize them to default initial values
       leftup = NULL;
       rightup = NULL;
       leftdown = NULL;
       rightdown = NULL;
       //Making the upper part of the conveyor belt
       
       {
   //defining each piece of the chain, along with the shape, size, fixture and relevant properties
         b2BodyDef bd;
   //The body is a dynamic body
         bd.type = b2_dynamicBody;
         bd.position.Set(conveyorleft, conveyorup);
         b2Body* ground = m_world->CreateBody(&bd);
         b2PolygonShape chainpiece;
   //Dimensions
         chainpiece.SetAsBox(1, 0.2); 
         b2FixtureDef chainfix;
   //Properties
         chainfix.density = 50;
         chainfix.friction = 0.5;
         chainfix.restitution = 0.5;
         chainfix.shape = &chainpiece;
   //Defining a revolute joint for our purpose
         b2Body *lastLink = ground;
         b2RevoluteJointDef chainJoint;
   //If the bodies are connected in the Box 2D world, then they should not collide. This is why this line is needed.
         chainJoint.collideConnected = false;

         for(int32 i = 0; i < links; i++)
         {
       double cons = 2.0;//This is the difference in length in between two anchors
       b2BodyDef bd;
       bd.type = b2_dynamicBody;
       bd.position.Set(i*cons + conveyorleft, 0.0 + conveyorup);//setting up a new dynamic chain link
       b2Body* nextLink = m_world->CreateBody(&bd);
       nextLink->CreateFixture(&chainfix);
       b2Vec2 anchor(i*cons - 1.0  + conveyorleft, 0.0 + conveyorup);//The anchor is set exactly at the meeting point of two chain links
       if(i > 0)//We should not define a link for the first piece alone. We need atleast two pieces to begin linking
       {
         chainJoint.Initialize(lastLink, nextLink, anchor);
         m_world->CreateJoint(&chainJoint);
       }
       //Updating the current link
       lastLink = nextLink;
       //assignment to the last link. The last link will essentially be rightdown, once we exit this loop
       rightdown = nextLink;
       //for the first link, we need to set leftdown, which is supposed to point to that link (body)
       if(i == 0)
       {
         leftdown = nextLink;
       }
     }
   }
       //here we make the lower part of the conveyor belt chain
   {
   double shiftup = 4.0;//this denotes the height to which we shift up the upper part of the chain
   // The rest of the process is identical to the first part except that everything is shifted up
   b2BodyDef bd;
   bd.type = b2_dynamicBody;
   bd.position.Set(conveyorleft, conveyorup + shiftup);
   b2Body* ground = m_world->CreateBody(&bd);
   b2PolygonShape chainpiece;
   chainpiece.SetAsBox(1, 0.2);
   b2FixtureDef chainfix;
   chainfix.density = 50;
   chainfix.friction = 0.5;
   chainfix.restitution = 0.5;
   chainfix.shape = &chainpiece;
   b2Body *lastLink = ground;
   b2RevoluteJointDef chainJoint;
   chainJoint.collideConnected = false;

   for(int32 i = 0; i < links; i++)
   {
     double cons = 2.0;
     b2BodyDef bd;
     bd.type = b2_dynamicBody;
     bd.position.Set(i*cons + conveyorleft, shiftup + conveyorup);
     b2Body* nextLink = m_world->CreateBody(&bd);
     nextLink->CreateFixture(&chainfix);
     b2Vec2 anchor(i*cons - 1.0 + conveyorleft, shiftup + conveyorup);
     if(i > 0)
     {
       chainJoint.Initialize(lastLink, nextLink, anchor);
       m_world->CreateJoint(&chainJoint);
     }
     lastLink = nextLink;
     rightup = nextLink;//"rightup" is updated to the right most link in the top part of the chain in every iteration
     if(i == 0)
     {
         leftup = nextLink;//"leftup" stores information about the topleft link
         //More Precisely, it stores a link to the body of the upper left link
       }
     }
   }

       //Now, we make the two links on the left hand side which connect the upper and lower halves of the chain
   {
     b2BodyDef bd;
   bd.type = b2_dynamicBody;//Note that this is a dynamic body
   //Position
   bd.position.Set(-1.0 + conveyorleft, conveyorup + 1.0);
   b2Body* ground = m_world->CreateBody(&bd);
   b2PolygonShape chainpiece;
   //Dimensions. Note that the dimensions are reversed, so that this is a vertical link, while the earlier one was a horizontal link
   chainpiece.SetAsBox(0.2, 1, b2Vec2(0.0,0.0),0.0);
   
   //Again defining a fixture for the chain
   b2FixtureDef chainfix;
   //Chain Properties
   chainfix.density = 50;
   chainfix.friction = 0.5;
   chainfix.restitution = 0.5;
   chainfix.shape = &chainpiece;
   //Creating the first vertical link on the left
   ground->CreateFixture(&chainfix);
   
   bd.position.Set(-1.0 + conveyorleft, conveyorup + 3.0);
   //Creating the second vertical link on the left
   b2Body*groundnew = m_world->CreateBody(&bd);
   groundnew->CreateFixture(&chainfix);
   
   //defining the revolute joint to connect links up
   b2RevoluteJointDef chainJoint;
   chainJoint.collideConnected = false;
   b2Vec2 anchor( conveyorleft - 1.0, conveyorup);
   //Connecting the lower left horizontal link and the lower vertical link
   chainJoint.Initialize(leftdown, ground, anchor);
   m_world->CreateJoint(&chainJoint);
   
   b2RevoluteJointDef chainJoint2;
   chainJoint2.collideConnected = false;
   b2Vec2 anchor2( conveyorleft - 1.0, 2.0 + conveyorup);
   //Connecting the lower and upper vertical links
   chainJoint2.Initialize(ground, groundnew, anchor2);
   m_world->CreateJoint(&chainJoint2);
   
   b2RevoluteJointDef chainJoint3;
   chainJoint3.collideConnected = false;
   b2Vec2 anchor3( conveyorleft - 1.0, 4.0 + conveyorup);
   //Connecting the upper vertical link and the top left horizontal link
   chainJoint3.Initialize(groundnew, leftup, anchor3);
   m_world->CreateJoint(&chainJoint3);
 }

       //Now again we connect the top and bottom portions of the conveyor belt, this time at the right end.
 {
   b2BodyDef bd;
   bd.type = b2_dynamicBody;
       //Note that the position is shifted towards the rights by a number proportional to the number of links in the chain
   bd.position.Set(2.0 * links - 1.0+ conveyorleft, conveyorup + 1.0);
   b2Body* ground = m_world->CreateBody(&bd);
   b2PolygonShape chainpiece;
   chainpiece.SetAsBox(0.2, 1, b2Vec2(0.0,0.0),0.0);
       //Defining the links and setting properties
   b2FixtureDef chainfix;
   chainfix.density = 50;
   chainfix.friction = 0.5;
   chainfix.restitution = 0.5;
   chainfix.shape = &chainpiece;
       //Making the lower vertical RHS link
   ground->CreateFixture(&chainfix);
     //Making the upper vertical RHS link
   bd.position.Set(2.0* links - 1.0+ conveyorleft, conveyorup + 3.0);
   b2Body*groundnew = m_world->CreateBody(&bd);
   groundnew->CreateFixture(&chainfix);
//Defining the revolute joint
   b2RevoluteJointDef chainJoint;
   chainJoint.collideConnected = false;
   b2Vec2 anchor( 2.0 * links - 1.0 + conveyorleft, 0.0 + conveyorup);
       //Joining the right most link of the lower chain and the lower vertical link on the right
   chainJoint.Initialize(rightdown, ground, anchor);
   m_world->CreateJoint(&chainJoint);

   b2RevoluteJointDef chainJoint2;
   chainJoint2.collideConnected = false;
   b2Vec2 anchor2( 2.0 * links - 1.0+ conveyorleft, 2.0 + conveyorup);
       //Joining the lower and upper right vertical links
   chainJoint2.Initialize(ground, groundnew, anchor2);
   m_world->CreateJoint(&chainJoint2);

   b2RevoluteJointDef chainJoint3;
   chainJoint3.collideConnected = false;
   b2Vec2 anchor3( 2.0 * links - 1.0+ conveyorleft, 4.0 + conveyorup);
       //joining the upper right vertical link with the top right horizontal link
   chainJoint3.Initialize(groundnew, rightup, anchor3);
   m_world->CreateJoint(&chainJoint3);
 }
       //Now, we create the rotating high friction discs
       //
       //The parameters are similar for all the discs, except that they are all translated along the x axis
//
//
//
 int numdiscs = links/5;
//The left most Disc:
 {
   b2BodyDef bd;
   bd.type = b2_kinematicBody;//This disc is a kinematic body
   bd.position.Set(conveyorleft + 0.0, conveyorup + 2.0);//Position
   bd.angle = 0;//starts rotating at angle 0
   b2Body* body = m_world->CreateBody(&bd);
   b2CircleShape circle;//shape of body = circular
   circle.m_radius = 2.0;//radius of disc used (same as the rectangular length of the boxes)
   
   b2FixtureDef fd;
   //properties of the disc
   fd.shape = &circle;
   fd.density = 100.0f;
   fd.friction = frction;//Note the friction. It is set as 1!
   body->CreateFixture(&fd);
   body->SetAngularVelocity(speed);
   
 }

       //The last disc (right most disc)
 {
   b2BodyDef bd;
   bd.type = b2_kinematicBody;
   //Note the shift in position from the earlier disc
   bd.position.Set(links * 2.0 + conveyorleft + -2.0, conveyorup + 2.0);
   bd.angle = 0;
   b2Body* body = m_world->CreateBody(&bd);
   b2CircleShape circle;
   circle.m_radius = 2.0;
   
   b2FixtureDef fd;
   fd.shape = &circle;
   fd.density = 100;
   fd.friction = frction;
   body->CreateFixture(&fd);
   body->SetAngularVelocity(speed);
 }


 for (int i = 1; i < numdiscs; ++i)
 {
  b2BodyDef bd;
  bd.type = b2_kinematicBody;
  //Note the position
  bd.position.Set(i * links/numdiscs * 2.0 + 1.0 + conveyorleft + -2.0, conveyorup + 2.0);
  bd.angle = 0;
  b2Body* body = m_world->CreateBody(&bd);
  b2CircleShape circle;
  circle.m_radius = 2.0;
  
  b2FixtureDef fd;
  fd.shape = &circle;
  fd.density = 100;
  fd.friction = 1;
  body->CreateFixture(&fd);
  body->SetAngularVelocity(speed);
}
{
  /* code */
}

//Spring-crank
{
  float a = -75.5f;
  float b = 0.f;

      //Fixed Box 1
  b2BodyDef *bd = new b2BodyDef;
  bd->position.Set(a+18.65f,b+10.1f);
  bd->fixedRotation = true;

  b2FixtureDef *fd1 = new b2FixtureDef;
  fd1->density = 10.0;
  fd1->friction = 0.5;
  fd1->shape = new b2PolygonShape;
  b2PolygonShape bs1;
  bs1.SetAsBox(0.8,2.2, b2Vec2(0.f,0.f), 0);
  fd1->shape = &bs1;

  b2Body* b4 = m_world->CreateBody(bd);
  b4->CreateFixture(fd1);

      // //Fixed Box 2
      // b2BodyDef *bc = new b2BodyDef;
      // bc->position.Set(a+25.35f,b+8.7f);
      // bc->fixedRotation = true;

      // b2FixtureDef *fc1 = new b2FixtureDef;
      // fc1->density = 10.0;
      // fc1->friction = 0.5;
      // fc1->shape = new b2PolygonShape;
      // b2PolygonShape bc1;
      // bc1.SetAsBox(0.2,0.7, b2Vec2(0.f,0.f), 0);
      // fc1->shape = &bc1;

      // b2Body* b3 = m_world->CreateBody(bc);
      // b3->CreateFixture(fc1);

//Bottom shelf
  b2BodyDef *bd1 = new b2BodyDef;
  bd1->position.Set(a+26.2f, b+7.6f);
  bd1->fixedRotation=true;

  b2FixtureDef *fd2 = new b2FixtureDef;
  fd2->density = 10.f;
  fd2->friction = 0.2f;
  fd2->shape = new b2PolygonShape;
  b2PolygonShape bs2;
  bs2.SetAsBox(14,0.1, b2Vec2(0.f,0.f), 0);
  fd2->shape = &bs2;

  b2Body* b5 = m_world->CreateBody(bd1);
  b5->CreateFixture(fd2);

//BOx

  b2BodyDef *bd2 = new b2BodyDef;
  bd2->type = b2_dynamicBody;
  bd2->position.Set(a+40.65f-1.5f+0.4f, b+11.0f);
      // bd2->position.Set(a+40.65f+3.0+0.4f, b+11.0f);
  bd2->fixedRotation = true;

      //The open box
  b2FixtureDef *fd5 = new b2FixtureDef;
  fd5->density = 10.0;
  fd5->friction = 0.5;
  fd5->restitution = 0.f;
  fd5->shape = new b2PolygonShape;
  b2PolygonShape bs5;
  bs5.SetAsBox(1.9,0.2, b2Vec2(0.f,-2.9f), 0);
  fd5->shape = &bs5;
  b2FixtureDef *fd3 = new b2FixtureDef;
  fd3->density = 10.0;
  fd3->friction = 0.5;
  fd3->restitution = 0.f;
  fd3->shape = new b2PolygonShape;
  b2PolygonShape bs3;
  bs3.SetAsBox(0.2,3, b2Vec2(1.9f,0.f), 0);
  fd3->shape = &bs3;
  b2FixtureDef *fd4 = new b2FixtureDef;
  fd4->density = 10.0;
  fd4->friction = 10.0;
  fd4->restitution = 0.f;
  fd4->shape = new b2PolygonShape;
  b2PolygonShape bs4;
  bs4.SetAsBox(0.2,3, b2Vec2(-1.9f,0.f), 0);
  fd4->shape = &bs4;

  b2Body* box1 = m_world->CreateBody(bd2);
  box1->CreateFixture(fd5);
  box1->CreateFixture(fd3);
  box1->CreateFixture(fd4);     

//Ball
  b2Body* spherebody;

  b2CircleShape circle;
  circle.m_radius = 0.75f;

  b2FixtureDef ball;
  ball.shape = &circle;
  ball.density = 50.0f;
  ball.friction = 0.2f;
  ball.restitution = 0.6f;

  b2BodyDef ballBody;
  ballBody.type = b2_dynamicBody;
  ballBody.position.Set(-39.2f,50.3);
  spherebody = m_world->CreateBody(&ballBody);
  spherebody->CreateFixture(&ball);

  {

      //0th Row Top plank
    b2Body* sbody00;
    b2PolygonShape poly00;
    b2Vec2 vertices00[4];
    vertices00[0].Set(-1.6f,0.8f);
    vertices00[1].Set(-1.7f,1.2f);
    vertices00[2].Set(1.7f,-0.8f);
    vertices00[3].Set(1.6f,-1.2f);
    poly00.Set(vertices00, 4);
    b2FixtureDef wedgefd00;
    wedgefd00.shape = &poly00;
    wedgefd00.density = 1.0f;
    wedgefd00.friction = 0.0f;
    wedgefd00.restitution = 0.0f;
    b2BodyDef wedgebd00;
    wedgebd00.type = b2_dynamicBody;
    wedgebd00.position.Set(a+21.05f,b+11.f);
    sbody00 = m_world->CreateBody(&wedgebd00);
    sbody00->CreateFixture(&wedgefd00);
    sbody00->ApplyForce( sbody00->GetMass() * -m_world->GetGravity(), sbody00->GetWorldCenter(), false );

    b2RevoluteJointDef Joint00;
    b2Vec2 anchor00;
    anchor00.Set(a+19.8f, b+12.0f);
    Joint00.collideConnected = true; 
    Joint00.Initialize(b4, sbody00, anchor00);
    m_world->CreateJoint(&Joint00);
      //19.35,7.8

      //0th Row Bottom plank
    b2Body* sbody01;
    b2PolygonShape poly01;
    b2Vec2 vertices01[4];
    vertices01[0].Set(-1.7f,-1.2f);
    vertices01[1].Set(-1.6f,-0.8f);
    vertices01[2].Set(1.6f,0.8f);
    vertices01[3].Set(1.7f,1.2f);
    poly01.Set(vertices01, 4);
    b2FixtureDef wedgefd01;
    wedgefd01.shape = &poly01;
    wedgefd01.density = 1.0f;
    wedgefd01.friction = 0.0f;
    wedgefd01.restitution = 0.0f;
    b2BodyDef wedgebd01;
    wedgebd01.type = b2_dynamicBody;
    wedgebd01.position.Set(a+21.05f,b+9.f);
    sbody01 = m_world->CreateBody(&wedgebd01);
    sbody01->CreateFixture(&wedgefd01);
    sbody01->ApplyForce( sbody01->GetMass() * -m_world->GetGravity(), sbody01->GetWorldCenter(), false );

    b2RevoluteJointDef Joint01;
    b2Vec2 anchor01;
    anchor01.Set(a+19.8f, b+8.0f);
    Joint01.collideConnected = true; 
    Joint01.Initialize(b4, sbody01, anchor01);
    m_world->CreateJoint(&Joint01);

    b2RevoluteJointDef Joint0001;
    b2Vec2 anchor0001;
    anchor0001.Set(a+22.6f, b+10.f);
    Joint0001.collideConnected = true; 
    Joint0001.Initialize(sbody00, sbody01, anchor0001);
    m_world->CreateJoint(&Joint0001); 

      // 1st Row top plank
    b2Body* sbody1;
    b2PolygonShape poly1;
    b2Vec2 vertices1[4];
    vertices1[0].Set(-1.55f,-0.8f);
    vertices1[1].Set(-1.45f,-1.2f);
    vertices1[2].Set(1.55f,0.8f);
    vertices1[3].Set(1.45f,1.2f);
    poly1.Set(vertices1, 4);
    b2FixtureDef wedgefd1;
    wedgefd1.shape = &poly1;
    wedgefd1.density = 1.0f;
    wedgefd1.friction = 0.0f;
    wedgefd1.restitution = 0.0f;
    b2BodyDef wedgebd1;
    wedgebd1.type = b2_dynamicBody;
    wedgebd1.position.Set(a+24.2f,b+11.f);
    sbody1 = m_world->CreateBody(&wedgebd1);
    sbody1->CreateFixture(&wedgefd1);
    sbody1->ApplyForce( sbody1->GetMass() * -m_world->GetGravity(), sbody1->GetWorldCenter(), true );


    b2RevoluteJointDef Joint001;
    b2Vec2 anchor001;
    anchor001.Set(a+22.7f, b+10.f);
    Joint001.collideConnected = true; 
    Joint001.Initialize(sbody00, sbody1, anchor001);
    m_world->CreateJoint(&Joint001);

      // b2RevoluteJointDef Joint2;
      // b2Vec2 anchor2;
      // anchor2.Set(a+22.8f, b+10.0f);
      // Joint2.collideConnected = true; 
      // Joint2.Initialize(b4, sbody1, anchor2);
      // m_world->CreateJoint(&Joint2);

      //1st Row Bottom plank
    b2Body* sbody2;
    b2PolygonShape poly2;
    b2BodyDef wedgebd2;
    wedgebd2.type = b2_dynamicBody;
    wedgebd2.position.Set(a+24.2f, b+9.f);
    b2Vec2 vertices2[4];
    vertices2[0].Set(-1.55f,1.2f);
    vertices2[1].Set(-1.45f,0.8f);
    vertices2[2].Set(1.45f,-1.2f);
    vertices2[3].Set(1.55f,-0.8f);
    poly2.Set(vertices2, 4);
    b2FixtureDef wedgefd2;
    wedgefd2.shape = &poly2;
    wedgefd2.density = 1.0f;
    wedgefd2.friction = 0.0f;
    wedgefd2.restitution = 0.0f;
    sbody2 = m_world->CreateBody(&wedgebd2);
    sbody2->CreateFixture(&wedgefd2);
    sbody2->ApplyForce( sbody2->GetMass() * -m_world->GetGravity(), sbody2->GetWorldCenter(), true );



    b2RevoluteJointDef Joint012;
    b2Vec2 anchor012;
    anchor012.Set(a+22.7f, b+10.0f);
    Joint012.collideConnected = true; 
    Joint012.Initialize(sbody2, sbody01, anchor012);
    m_world->CreateJoint(&Joint012);

      // b2RevoluteJointDef Joint1;
      // b2Vec2 anchor1;
      // anchor1.Set(a+22.8f, b+10.0f);
      // Joint1.collideConnected = true; 
      // Joint1.Initialize(b4, sbody, anchor1);
      // m_world->CreateJoint(&Joint1);

      //2nd Row Top plank
    b2Body* sbody3;
    b2PolygonShape poly3;
    b2Vec2 vertices3[4];
    vertices3[0].Set(-1.6f,0.8f);
    vertices3[1].Set(-1.5f,1.2f);
    vertices3[2].Set(1.6f,-0.8f);
    vertices3[3].Set(1.5f,-1.2f);
    poly3.Set(vertices3, 4);
    b2FixtureDef wedgefd3;
    wedgefd3.shape = &poly3;
    wedgefd3.density = 1.0f;
    wedgefd3.friction = 0.0f;
    wedgefd3.restitution = 0.0f;
    b2BodyDef wedgebd3;
    wedgebd3.type = b2_dynamicBody;
    wedgebd3.position.Set(a+27.15f,b+11.f);
    sbody3 = m_world->CreateBody(&wedgebd3);
    sbody3->CreateFixture(&wedgefd3);
    sbody3->ApplyForce( sbody3->GetMass() * -m_world->GetGravity(), sbody3->GetWorldCenter(), false );

    b2RevoluteJointDef Joint13;
    b2Vec2 anchor13;
    anchor13.Set(a+25.6f, b+12.f);
    Joint13.collideConnected = true; 
    Joint13.Initialize(sbody1, sbody3, anchor13);
    m_world->CreateJoint(&Joint13);

      //2nd Row Bottom plank
    b2Body* sbody4;
    b2PolygonShape poly4;
    b2Vec2 vertices4[4];
    vertices4[0].Set(-1.5f,-1.2f);
    vertices4[1].Set(-1.6f,-0.8f);
    vertices4[2].Set(1.5f,0.8f);
    vertices4[3].Set(1.6f,1.2f);
    poly4.Set(vertices4, 4);
    b2FixtureDef wedgefd4;
    wedgefd4.shape = &poly4;
    wedgefd4.density = 1.0f;
    wedgefd4.friction = 0.0f;
    wedgefd4.restitution = 0.0f;
    b2BodyDef wedgebd4;
    wedgebd4.type = b2_dynamicBody;
    wedgebd4.position.Set(a+27.15f,b+9.f);
    sbody4 = m_world->CreateBody(&wedgebd4);
    sbody4->CreateFixture(&wedgefd4);
    sbody4->ApplyForce( sbody4->GetMass() * -m_world->GetGravity(), sbody4->GetWorldCenter(), false );

    b2RevoluteJointDef Joint24;
    b2Vec2 anchor24;
    anchor24.Set(a+25.6f, b+8.f);
    Joint24.collideConnected = true; 
    Joint24.Initialize(sbody4, sbody2, anchor24);
    m_world->CreateJoint(&Joint24);

    b2RevoluteJointDef Joint34;
    b2Vec2 anchor34;
    anchor34.Set(a+28.6f, b+10.f);
    Joint34.collideConnected = true; 
    Joint34.Initialize(sbody4, sbody3, anchor34);
    m_world->CreateJoint(&Joint34);      

      //3rd Row top plank
    b2Body* sbody5;
    b2PolygonShape poly5;
    b2Vec2 vertices5[4];
    vertices5[0].Set(-1.5f,-1.2f);
    vertices5[1].Set(-1.6f,-0.8f);
    vertices5[3].Set(1.6f,0.8f);
    vertices5[2].Set(1.5f,1.2f);
    poly5.Set(vertices5, 4);
    b2FixtureDef wedgefd5;
    wedgefd5.shape = &poly5;
    wedgefd5.density = 1.0f;
    wedgefd5.friction = 0.0f;
    wedgefd5.restitution = 0.0f;
    b2BodyDef wedgebd5;
    wedgebd5.type = b2_dynamicBody;
    wedgebd5.position.Set(a+30.15f,b+11.f);
    sbody5 = m_world->CreateBody(&wedgebd5);
    sbody5->CreateFixture(&wedgefd5);
    sbody5->ApplyForce( sbody5->GetMass() * -m_world->GetGravity(), sbody5->GetWorldCenter(), true );

    b2RevoluteJointDef Joint35;
    b2Vec2 anchor35;
    anchor35.Set(a+28.6f, b+10.f);
    Joint35.collideConnected = true; 
    Joint35.Initialize(sbody5, sbody3, anchor35);
    m_world->CreateJoint(&Joint35);

      //3rd Row Bottom plank
    b2Body* sbody6;
    b2PolygonShape poly6;
    b2BodyDef wedgebd6;
    wedgebd6.type = b2_dynamicBody;
    wedgebd6.position.Set(a+30.15f, b+9.f);
    b2Vec2 vertices6[4];
    vertices6[0].Set(-1.5f,1.2f);
    vertices6[1].Set(-1.6f,0.8f);
    vertices6[2].Set(1.5f,-1.2f);
    vertices6[3].Set(1.6f,-0.8f);
    poly6.Set(vertices6, 4);
    b2FixtureDef wedgefd6;
    wedgefd6.shape = &poly6;
    wedgefd6.density = 1.0f;
    wedgefd6.friction = 0.0f;
    wedgefd6.restitution = 0.0f;
    sbody6 = m_world->CreateBody(&wedgebd6);
    sbody6->CreateFixture(&wedgefd6);
    sbody6->ApplyForce( sbody6->GetMass() * -m_world->GetGravity(), sbody6->GetWorldCenter(), true );

    b2RevoluteJointDef Joint46;
    b2Vec2 anchor46;
    anchor46.Set(a+28.6f, b+10.0f);
    Joint46.collideConnected = true; 
    Joint46.Initialize(sbody6, sbody4, anchor46);
    m_world->CreateJoint(&Joint46);



      //4th Row Top plank
    b2Body* sbody7;
    b2PolygonShape poly7;
    b2Vec2 vertices7[4];
    vertices7[0].Set(-1.55f,0.8f);
    vertices7[1].Set(-1.45f,1.2f);
    vertices7[2].Set(1.55f,-0.8f);
    vertices7[3].Set(1.45f,-1.2f);
    poly7.Set(vertices7, 4);
    b2FixtureDef wedgefd7;
    wedgefd7.shape = &poly7;
    wedgefd7.density = 1.0f;
    wedgefd7.friction = 0.0f;
    wedgefd7.restitution = 0.0f;
    b2BodyDef wedgebd7;
    wedgebd7.type = b2_dynamicBody;
    wedgebd7.position.Set(a+33.1f,b+11.f);
    sbody7 = m_world->CreateBody(&wedgebd7);
    sbody7->CreateFixture(&wedgefd7);
    sbody7->ApplyForce( sbody7->GetMass() * -m_world->GetGravity(), sbody7->GetWorldCenter(), false );

    b2RevoluteJointDef Joint57;
    b2Vec2 anchor57;
    anchor57.Set(a+31.6f, b+12.f);
    Joint57.collideConnected = true; 
    Joint57.Initialize(sbody7, sbody5, anchor57);
    m_world->CreateJoint(&Joint57);

      //4thRow Bottom plank
    b2Body* sbody8;
    b2PolygonShape poly8;
    b2Vec2 vertices8[4];
    vertices8[0].Set(-1.45f,-1.2f);
    vertices8[1].Set(-1.55f,-0.8f);
    vertices8[2].Set(1.45f,0.8f);
    vertices8[3].Set(1.55f,1.2f);
    poly8.Set(vertices8, 4);
    b2FixtureDef wedgefd8;
    wedgefd8.shape = &poly8;
    wedgefd8.density = 1.0f;
    wedgefd8.friction = 0.0f;
    wedgefd8.restitution = 0.0f;
    b2BodyDef wedgebd8;
    wedgebd8.type = b2_dynamicBody;
    wedgebd8.position.Set(a+33.1f,b+9.f);
    sbody8 = m_world->CreateBody(&wedgebd8);
    sbody8->CreateFixture(&wedgefd8);
    sbody8->ApplyForce( sbody8->GetMass() * -m_world->GetGravity(), sbody8->GetWorldCenter(), false );

    b2RevoluteJointDef Joint68;
    b2Vec2 anchor68;
    anchor68.Set(a+31.6f, b+8.f);
    Joint68.collideConnected = true; 
    Joint68.Initialize(sbody8, sbody6, anchor68);
    m_world->CreateJoint(&Joint68);

    b2RevoluteJointDef Joint78;
    b2Vec2 anchor78;
    anchor78.Set(a+34.6f, b+10.f);
    Joint78.collideConnected = true; 
    Joint78.Initialize(sbody7, sbody8, anchor78);
    m_world->CreateJoint(&Joint78);   


      //5th Row top plank
    b2Body* sbody9;
    b2PolygonShape poly9;
    b2Vec2 vertices9[4];
    vertices9[0].Set(-1.5f,-1.2f);
    vertices9[1].Set(-1.6f,-0.8f);
    vertices9[3].Set(1.6f,0.8f);
    vertices9[2].Set(1.5f,1.2f);
    poly9.Set(vertices9, 4);
    b2FixtureDef wedgefd9;
    wedgefd9.shape = &poly9;
    wedgefd9.density = 1.0f;
    wedgefd9.friction = 10.0f;
    wedgefd9.restitution = 0.0f;
    b2BodyDef wedgebd9;
    wedgebd9.type = b2_dynamicBody;
    wedgebd9.position.Set(a+36.15f,b+11.f);
    sbody9 = m_world->CreateBody(&wedgebd9);
    sbody9->CreateFixture(&wedgefd9);
    sbody9->ApplyForce( sbody9->GetMass() * -m_world->GetGravity(), sbody9->GetWorldCenter(), true );

    b2RevoluteJointDef Joint79;
    b2Vec2 anchor79;
    anchor79.Set(a+34.6f, b+10.f);
    Joint79.collideConnected = true; 
    Joint79.Initialize(sbody7, sbody9, anchor79);
    m_world->CreateJoint(&Joint79);

      //5th Row Bottom plank
    b2Body* sbody10;
    b2PolygonShape poly10;
    b2BodyDef wedgebd10;
    wedgebd10.type = b2_dynamicBody;
    wedgebd10.position.Set(a+36.15f, b+9.f);
    b2Vec2 vertices10[4];
    vertices10[0].Set(-1.5f,1.2f);
    vertices10[1].Set(-1.6f,0.8);
    vertices10[2].Set(1.5f,-1.2f);
    vertices10[3].Set(1.6f,-0.8f);
    poly10.Set(vertices10, 4);
    b2FixtureDef wedgefd10;
    wedgefd10.shape = &poly10; 
    wedgefd10.density = 1.0f;
    wedgefd10.friction = 0.0f;
    wedgefd10.restitution = 0.0f;
    sbody10 = m_world->CreateBody(&wedgebd10);
    sbody10->CreateFixture(&wedgefd10);
    sbody10->ApplyForce( sbody10->GetMass() * -m_world->GetGravity(), sbody10->GetWorldCenter(), true );

    b2RevoluteJointDef Joint810;
    b2Vec2 anchor810;
    anchor810.Set(a+34.6f, b+10.0f);
    Joint810.collideConnected = true; 
    Joint810.Initialize(sbody8, sbody10, anchor810);
    m_world->CreateJoint(&Joint810);

  }


}
}
}

sim_t *sim = new sim_t("Dominos", dominos_t::create);
}