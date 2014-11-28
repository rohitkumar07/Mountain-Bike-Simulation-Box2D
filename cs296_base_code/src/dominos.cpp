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
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */

/*
*\struct to document a C-struct.
*\union to document a union.
*\enum to document an enumeration type.
*\fn to document a function.
*\var to document a variable or typedef or enum value.
*\def to document a #define.
*\typedef to document a type definition.
*\file to document a file.
*\namespace to document a namespace.
*\package to document a Java package.
*\interface to document an IDL interface.
*/
#include <iostream>
#include "cs296_base.hpp"
#include "render.hpp"
#include <cmath>

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs296
{
  /*!  This is the constructor 
   * This is the documentation block for the constructor.
   */ 
    b2Body* sbody1;
    b2Body* body;
    b2Body* gear1;
    b2Body* gear2;
    b2Body* gear3;
    b2Body* body2;
    b2Body* frontWheelBody;

    int gearIndex = 1;

  dominos_t::dominos_t()
  {
    //gearIndex = 1;
	 /*! \section sec Elements of Graphics
	 * 	
	 */ 
    
    /*! \subsection Ground
     * 
     * \brief Ground is the base of all the elements in the cycle. Cycle simulation moves considering ground as a surface.
     * 
     * \li b1 is pointer of \a b2Body type, set to an edge shape using b2EdgeShape which is set to a line between 
     * point (-450,0) and (450,0).
     * 
     * \li \a bd is \a b2BodyDef type which stores the data about ground and then creates it.
     * \n \n
     */
     
     //Ground
     
    b2Body* b1;
    { 
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-450.0f, 0.0f), b2Vec2(450.0f, 0.0f));
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }

    //The see-saw system at the bottom
    
    
    /*! \subsection tri Triangle wedge
     * 
     * \li This part creates a triangular wedge for resovling up-down simulation of cycle while colliding with wedge.
     * \li \a wedgebd is \a b2BodyDef type which stores the data about wedge-structure and then creates it.
     * \li \a sbody is a pointer of type \a b2Body. \a poly is an object of class \a b2PolygonShape. It has 3 vertices,
     *  position of which is set to (-1,0)(1,0)(0,1.5) by calling function \a set on each vertex.
     * Its density is 10. It is frictionless and perfectly inelastic. Position of \a wedgebd is set to (30, 0)
     *  using function \a position.set().
     * \n \n
     * 
     */
     
     
    {
      //The triangle wedge
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,1.5);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(70.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
      }

          {
      //The triangle wedge
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(0,0);
      vertices[1].Set(100,0);
      vertices[2].Set(100,2);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(130.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
      
      }

  /// Joint Movements

  /// The Chain System

  
  //short CATEGORY_NOTCOLLIDE = 0x0001;

  
  /// The rear wheel
  /*! \subsection rear Rear Wheel-Structure
   * 
   * \li \a bd2 is the bodydefination about the rear-wheel body named as \a body2 and \a fdBackWheelFixture is an pointer to fixture 
   * defination to rear-wheel. 
   * \li \a body2 is in 10 radius circular shape as described in \a shape2 defination and is a dynamic body positioned in (-17,10)
   *  as described in \a bd2.
   * 
   */
      
      
      b2CircleShape shape2;
      shape2.m_radius = 10.0f;
      b2BodyDef bd2;
      bd2.position.Set(-17.0f, 10.0f);
      bd2.type = b2_dynamicBody;

      body2 = m_world->CreateBody(&bd2);
      
      b2FixtureDef *fdBackWheelFixture = new b2FixtureDef;
      {
        fdBackWheelFixture->density = 1.0f;
        fdBackWheelFixture->shape = new b2CircleShape;
        fdBackWheelFixture->shape = &shape2;
        fdBackWheelFixture->filter.groupIndex = -1;
        fdBackWheelFixture->friction = 30.0f;
      }

      body2->CreateFixture(fdBackWheelFixture);
      /// Gear1
	  /*! \subsection gear1 Gear1 Axle and joint
	   * \li This part creates 1st gear-axle named as \a gear1 and weld it with the rear-wheel \a body2 using weldjoint.
	   * \li \a gear1 is also in circular shape with radius 4 as described in shapeGear1, an element of class b2CircleShape.
	   * \li \a fdGear1 is a pointer to b2FixtureDef class which assigns density, shape and filtering bits to avoid collision with 
	   * rear-wheel.   
	   * \li \a gear1 is positined in (-17,10) same as rear-wheel centre as described in \a bdGear.
	   * \li Joint - \a gear1 jointed with \a body2 at their common centre using \a jointDefGear1 as an object of b2WeldJointDef class.
	   */
	  b2BodyDef bdGear;
      bdGear.position.Set(-17.0f, 10.0f);
      bdGear.type = b2_dynamicBody;
      b2CircleShape shapeGear1;
      shapeGear1.m_radius = 4.0f;
  
      gear1 = m_world->CreateBody(&bdGear);

      b2FixtureDef *fdGear1 = new b2FixtureDef;
      {
        fdGear1->density = 10.0f;
        fdGear1->shape = new b2PolygonShape;
        fdGear1->shape = &shapeGear1;

        fdGear1->filter.categoryBits = 0x0002;
        fdGear1->filter.maskBits = 0x0004;
      }
      gear1->CreateFixture(fdGear1);

    

      b2WeldJointDef jointDefGear1;
      jointDefGear1.bodyA = gear1;
      jointDefGear1.bodyB = body2;

      jointDefGear1.localAnchorA.Set(0,0);
      jointDefGear1.localAnchorB.Set(0,0);
      jointDefGear1.collideConnected = false;
      m_world->CreateJoint(&jointDefGear1);
  

  /// gear2
  /*! \subsection gear2 Gear2 Axle and joint
	   * \li This part creates 1st gear-axle named as \a gear2 and weld it with the rear-wheel \a body2 using weldjoint.
	   * \li \a gear1 is also in circular shape with radius 3 as described in shapeGear2, an element of class b2CircleShape.
	   * \li \a fdGear2 is a pointer to b2FixtureDef class which assigns density, shape and filtering bits to avoid collision with 
	   * rear-wheel.   
	   * \li \a gear2 is positined in (-17,10) same as rear-wheel centre as described in \a bdGear as an object of b2BodyDef class.
	   * \li Joint - \a gear2 jointed with \a body2 at their common centre using \a jointDefGear2 as an object of b2WeldJointDef class.
	   * 
	   */
      b2CircleShape shapeGear2;
      shapeGear2.m_radius = 3.0f;
      
      gear2 = m_world->CreateBody(&bdGear);

      b2FixtureDef *fdGear2 = new b2FixtureDef;
      {
        fdGear2->density = 10.0f;
        fdGear2->shape = new b2PolygonShape;
        fdGear2->shape = &shapeGear2;

        fdGear2->filter.categoryBits = 0x0002;
        fdGear2->filter.maskBits = 0x0004;
      }
      gear2->CreateFixture(fdGear2);

      b2WeldJointDef jointDefGear2;
      jointDefGear2.bodyA = gear2;
      jointDefGear2.bodyB = body2;

      jointDefGear2.localAnchorA.Set(0,0);
      jointDefGear2.localAnchorB.Set(0,0);
      jointDefGear2.collideConnected = false;
      m_world->CreateJoint(&jointDefGear2);
  
  
  /// gear3
  /*! \subsection gear3 Gear3 Axle and joint
	   * \li This part creates 1st gear-axle named as \a gear3 and weld it with the rear-wheel \a body2 using weldjoint.
	   * \li \a gear3 is also in circular shape with radius 2.5 as described in shapeGear3, an element of class b2CircleShape.
	   * \li \a fdGear3 is a pointer to b2FixtureDef class which assigns density, shape and filtering bits to avoid collision with 
	   * rear-wheel.   
	   * \li \a gear3 is positined in (-17,10) same as rear-wheel centre as described in \a bdGear.
	   * \li Joint - \a gear3 jointed with \a body2 at their common centre using \a jointDefGear3 as an object of b2WeldJointDef class.
	   * 
	   */
      b2CircleShape shapeGear3;
      shapeGear3.m_radius = 2.5f;

      gear3 = m_world->CreateBody(&bdGear);
      b2FixtureDef *fdGear3 = new b2FixtureDef;
      {
        fdGear3->density = 10.0f;
        fdGear3->shape = new b2PolygonShape;
        fdGear3->shape = &shapeGear3;
        fdGear3->filter.categoryBits = 0x0002;
        fdGear3->filter.maskBits = 0x0004;
      }

      gear3->CreateFixture(fdGear3);

      b2WeldJointDef jointDefGear3;
      jointDefGear3.bodyA = gear3;
      jointDefGear3.bodyB = body2;

      jointDefGear3.localAnchorA.Set(0,0);
      jointDefGear3.localAnchorB.Set(0,0);
      jointDefGear3.collideConnected = false;
      m_world->CreateJoint(&jointDefGear3);

/// gear Rod1
/*! \subsection gear-systemrod1 Gear Supporting Rod1
	   * \li Rod named as \a axleBody1 is used to support small pully attached to rear axle.While changing gear this rod rotates about the hinge point 
	   *  and creates necessary change in chain length and thus causing change in gear.
	   * \li \a axleBody1 is shaped like a rod with dimension (6,1) as described in \a axle1 and positioned such that one end 
	   * coincides with the rear-wheel centre and rod is rotated delta angle in clockwise direction using SetTransform.  
	   * 
	   */
      b2PolygonShape axle1;
      axle1.SetAsBox(3.0f, 0.5f);

      float delta = b2_pi/6;

      b2BodyDef bd3Axle1;
      bd3Axle1.position.Set(-17.0f + (3.0f * cos(delta)), 10.0f - (3.0f* sin(delta)) );
      bd3Axle1.type = b2_dynamicBody;
      b2Body* axleBody1 = m_world->CreateBody(&bd3Axle1);

      b2FixtureDef *fdaxle1 = new b2FixtureDef;
      fdaxle1->density = 1.0f;
      fdaxle1->shape = new b2PolygonShape;
      fdaxle1->shape = &axle1;
      

      fdaxle1->filter.groupIndex = -1;
      axleBody1->CreateFixture(fdaxle1);
 
      axleBody1->SetTransform(axleBody1->GetPosition(), -1*delta);

      

/// Small Gear Wheel 1
/*! \subsection smallwheel1 Small Gear Wheel 1
	   * \li Wheel named as \a axleWheel1 is used to provide smooth pathway for chain.This wheel is joint to rod \a axleBody1, and while
	   * changing gear rod rotates and changes the position of wheel further creating necessary slack in chain.
	   * \li \a axleWheel1 is in circular shape with radius 1 as described in \a shapeaxle and positioned such that its center 
	   * coincides with outter rod corner of \a axleBody1.
	   * \li Joint - \a axleWheel1 is hinged with \a axleBody1 using revolutejoint about the centre of the wheel using \a jointWheel as 
	   * an object of b2RevoluteJointDef. 
	   * 
	   */
      b2CircleShape shapeAxle;
      shapeAxle.m_radius = 1.0f;
  
      b2BodyDef bdWheel;
      bdWheel.position.Set(-17.0f + (6.0f * cos(delta)), 10.0f - (6.0f * sin(delta)));
      bdWheel.type = b2_dynamicBody;
      
      b2Body* axleWheel1 = m_world->CreateBody(&bdWheel);

      b2FixtureDef *fdWheel = new b2FixtureDef;
      fdWheel->density = 10.f;
      fdWheel->shape = new b2PolygonShape;
      fdWheel->shape = &shapeAxle;

      fdWheel->filter.categoryBits = 0x0002;
      fdWheel->filter.maskBits = 0x0004;

      axleWheel1->CreateFixture(fdWheel);

      b2RevoluteJointDef jointWheel;

      jointWheel.bodyA = axleBody1;
      jointWheel.bodyB = axleWheel1;
      jointWheel.localAnchorA.Set(3.0f,0);
      jointWheel.localAnchorB.Set(0,0);
      jointWheel.collideConnected = false;
      m_world->CreateJoint(&jointWheel);


/// gear Rod2
/*! \subsection gear-systemrod2 Gear Supporting Rod2
	   * \li Rod named as \a axleBody2 is used to support small pully \a axleWheel2.While changing gear this rod rotates about the 
	   * hinge point and creates necessary change in chain length and thus causing change in gear.
	   * \li \a axleBody2 is shaped like a rod with dimension (4,1) as described in \a axle2 and positioned such that one end 
	   * coincides with the previous hinge point of \a axleWheel1 and \a axleBody1 and rod is rotated delta - pie angle in 
	   * anti-clockwise direction using SetTransform.  
	   * \li Joint- Both \a axleBody1 and axleBody2 are joint using revolutejointdef \a jointAxle2 at their common hinge point.
	   */
      b2PolygonShape axle2;
      axle2.SetAsBox(2.0f, 0.5f);


      b2BodyDef bd3Axle2;
      bd3Axle2.position.Set(axleWheel1->GetPosition().x - (2.0f * sin(delta)), axleWheel1->GetPosition().y - (2.0f* cos(delta)));
      bd3Axle2.type = b2_dynamicBody;
      b2Body* axleBody2 = m_world->CreateBody(&bd3Axle2);

      b2FixtureDef *fdaxle2 = new b2FixtureDef;
      fdaxle2->density = 1.0f;
      fdaxle2->shape = new b2PolygonShape;
      fdaxle2->shape = &axle2;
      

      fdaxle2->filter.groupIndex = -1;
      axleBody2->CreateFixture(fdaxle2);
 
      axleBody2->SetTransform(axleBody2->GetPosition(), delta - b2_pi);

      b2RevoluteJointDef jointAxle2;

      jointAxle2.bodyA = axleBody2;
      jointAxle2.bodyB = axleBody1;
      jointAxle2.localAnchorA.Set(-2.0, 0.0);
      jointAxle2.localAnchorB.Set(3.0,0);
      jointAxle2.upperAngle = b2_pi/48 + b2_pi/2;
      jointAxle2.lowerAngle = -1*b2_pi/48 + b2_pi/2;
      jointAxle2.enableLimit = true;
      jointAxle2.collideConnected = false;
      m_world->CreateJoint(&jointAxle2);

/// Small Gear Wheel 2
/*! \subsection smallwheel2 Small Gear Wheel 2
	   * \li Wheel named as \a axleWheel2 is used to provide smooth pathway for chain.This wheel is joint to rod \a axleBody2, and while
	   * changing gear rod \a axleBody2 rotates and changes the position of wheel further creating necessary slack in chain.
	   * \li \a axleWheel2 is in circular shape with radius 1 as described in \a fdWheel and positioned such that its center 
	   * coincides with outter rod corner of rod \a axleBody2.
	   * \li Joint - \a axleWheel2 is hinged with \a axleBody2 using revolutejoint about the centre of the wheel using \a jointWheel2 as 
	   * an object of b2RevoluteJointDef. 
	   * 
	   */
  
      b2BodyDef bdWheel2;
      bdWheel2.position.Set(axleWheel1->GetPosition().x - (4.0f * sin(delta)), axleWheel1->GetPosition().y - (4.0f* cos(delta)));
      bdWheel2.type = b2_dynamicBody;
      
      b2Body* axleWheel2 = m_world->CreateBody(&bdWheel2);

      axleWheel2->CreateFixture(fdWheel);

      b2RevoluteJointDef jointWheel2;

      jointWheel2.bodyA = axleBody2;
      jointWheel2.bodyB = axleWheel2;
      jointWheel2.localAnchorA.Set(2.0f,0);
      jointWheel2.localAnchorB.Set(0,0);
      jointWheel2.collideConnected = false;
      m_world->CreateJoint(&jointWheel2);
/*
      b2DistanceJointDef jointGear;
      jointGear.bodyA = axleWheel2;
      jointGear.bodyB = gear3;
      jointGear.localAnchorA.Set(-6.0f,0);
      jointGear.localAnchorB.Set(6.0f,0);
      jointGear.collideConnected = true;
      m_world->CreateJoint(&jointGear);
      jointGear.frequencyHz = 4.0f;
      jointGear.dampingRatio = 0.5f;
*/
  

/// The pedal wheel
/*! \subsection padelwheel Pedal Axle
	   * \li Axle named as \a body is used as the padel wheel and chain rolls smoothly over this wheel.
	   * \li \a body is in circular shape with radius 3 as described in \a shape and positioned at (-2,10) as mentioned in \a bd an 
	   * object of type b2BodyDef.
	   * \li \a body is assigned density 10 and appropriate filter bits for grouping different bodies as mentioned in \a fd a pointer 
	   * to b2FixtureDef class object.
	   * 
	   */

      b2CircleShape shape;
      shape.m_radius = 3.0f;
      
  
      b2BodyDef bd;
      bd.position.Set(-2.0f, 10.0f);
      bd.type = b2_dynamicBody;
      //bd.type = b2_kinematicBody;

      body = m_world->CreateBody(&bd);

      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 10.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      fd->filter.categoryBits = 0x0002;
      fd->filter.maskBits = 0x0004;

      body->CreateFixture(fd);
      /*
      body->SetAngularVelocity(-2.0f);
      body->SetLinearVelocity(b2Vec2(2.0f,0.0f));
      */
      //body->b2Color(&col);



/// Rod connecting pedal wheel to back of bicycle
/*! \subsection supporting-rod Pedal axle supporting rod
	   * \li Rod named as \a newBodyBack is used to support pedal axle \a body by connecting one end of rod to center of pedal axle 
	   * and another end to rear wheel center.
	   * \li \a newBodyBack is shaped like a rectangle with dimension (15,0.6) as described in \a back and positioned at (-9.5,10) such
	   *  that one end coincides with the center of rear wheel \a body2 and another end coincides with center of padel-axle \a body.
	   * \li Joint1- Rod \a newBodyBack and pedal-axle \a body are joined using revolutejointdef \a jointDefBack at pedal-axle center.
	   * \li Joint2- Rod \a newBodyBack and 3rd gear \a gear3 are joined using revolutejointdef \a jointGearBack at gear3 axle center.
	   * \li Joint3- Rod \a newBodyBack and supporting rod \a axleBody1 are joined using revolutejointdef \a jointAxle1 at pedal-axle 
	   * center and angle limits are assigned such that \a axleBody1 can rotate under constraint while changing gears.
	   * 
	   */
      b2PolygonShape back;
      back.SetAsBox(7.5f, 0.3f);

      
      b2BodyDef bd3Back;
      bd3Back.position.Set(-9.5f, 10.0f);
      bd3Back.type = b2_dynamicBody;
      b2Body* newBodyBack = m_world->CreateBody(&bd3Back);
      
      b2FixtureDef *fdBack = new b2FixtureDef;
      fdBack->density = 1.0f;
      fdBack->shape = new b2PolygonShape;
      fdBack->shape = &back;
      
      fdBack->filter.groupIndex = -1;
      newBodyBack->CreateFixture(fdBack);
      

      b2RevoluteJointDef jointDefBack, jointGearBack, jointAxle1;
      jointDefBack.bodyA = body;
      jointDefBack.bodyB = newBodyBack;
      jointDefBack.localAnchorA.Set(0,0);
      jointDefBack.localAnchorB.Set(7.5,0);
      jointDefBack.collideConnected = false;
      m_world->CreateJoint(&jointDefBack);

      jointGearBack.bodyA = gear3;
      jointGearBack.bodyB = newBodyBack;
      jointGearBack.localAnchorA.Set(0,0);
      jointGearBack.localAnchorB.Set(-7.5,0);
      jointGearBack.collideConnected = false;
      m_world->CreateJoint(&jointGearBack);

      jointAxle1.bodyA = axleBody1;
      jointAxle1.bodyB = newBodyBack;
      jointAxle1.localAnchorA.Set(-3.0, 0.0);
      jointAxle1.localAnchorB.Set(-7.5,0);
      jointAxle1.upperAngle = b2_pi/12 + b2_pi/6;
      jointAxle1.lowerAngle = -1*b2_pi/96 + b2_pi/6;
      jointAxle1.enableLimit = true;
      jointAxle1.collideConnected = false;
      m_world->CreateJoint(&jointAxle1);


/// Rod connecting pedal wheel to front of bicycle
/*! \subsection supporting-rod2 Pedal axle to front cycle supporting rod
	   * \li Rod named as \a newBodyFront is used to support pedal axle \a body by connecting one end of rod to center of pedal axle 
	   * and another end to front part of the cycle.
	   * \li \a newBodyFront is shaped like a rectangle with dimension (20,0.6) as described in \a front and positioned such that one 
	   * end coincides with center of padel-axle \a body and rod is set at an angle \a frontTheta anti-clockwise using SetTransform.
	   * \li Joint- Rod \a newBodyBack and rod \a newBodyFront are joined using revolutejointdef \a jointDefFront at pedal-axle center
	   * and are fixed by assigning same value to upper and lower angle in joint defination.
	   * 
	   */
      b2PolygonShape front;
      front.SetAsBox(10.0f, 0.3f);

      float frontTheta = b2_pi/6;
      b2BodyDef bd3Front;
      bd3Front.position.Set(-2.0f + (cos(frontTheta) * 10.0f), 10.0f + (sin(frontTheta) * 10.0f));
      bd3Front.type = b2_dynamicBody;
      b2Body* newBodyFront = m_world->CreateBody(&bd3Front);
      
      b2FixtureDef *fdFront = new b2FixtureDef;
      fdFront->density = 1.0f;
      fdFront->shape = new b2PolygonShape;
      fdFront->shape = &front;
      
      fdFront->filter.groupIndex = -1;
      newBodyFront->CreateFixture(fdFront);
      newBodyFront->SetTransform(newBodyFront->GetPosition(), frontTheta);

      b2RevoluteJointDef jointDefFront;
      jointDefFront.bodyA = newBodyBack;
      jointDefFront.bodyB = newBodyFront;
      jointDefFront.localAnchorA.Set(7.5f,0);
      jointDefFront.localAnchorB.Set(-10.0,0);
      jointDefFront.upperAngle = frontTheta;
      jointDefFront.lowerAngle = frontTheta;
      jointDefFront.enableLimit = true;
      jointDefFront.collideConnected = false;
      m_world->CreateJoint(&jointDefFront);

/// Rod to front Wheel
/*! \subsection supporting-rod3 Front Wheel Rod
	   * \li Rod named as \a newBodyFrontRod is used to provide rigid structure to front part of cycle and also used as handle of cycle.
	   * \li \a newBodyFrontRod is shaped like a rectangle with dimension (20*tan(\a frontTheta),0.6) as described in \a frontRod and 
	   * positioned such that one end coincides with outter end of \a newBodyFront and another end to be attached to front-wheel center
	   * \li Rod is set at an angle pi/2 - \a frontTheta anti-clockwise using SetTransform.
	   * \li Joint- Rod \a newBodyFrontRod and rod \a newBodyFront are joined using revolutejointdef \a jointDefFrontRod at their 
	   * common overlapping point and are fixed by assigning same value to upper and lower angle in joint defination.
	   * 
	   */
      b2PolygonShape frontRod;
      frontRod.SetAsBox(10.0f * tan(frontTheta), 0.3f);

      b2BodyDef bd3FrontRod;
      bd3FrontRod.position.Set(-2.0f + (10.0f/sin(frontTheta)), 10.0f + (sin(frontTheta) * 10.0f));
      bd3FrontRod.type = b2_dynamicBody;
      b2Body* newBodyFrontRod = m_world->CreateBody(&bd3FrontRod);
      
      b2FixtureDef *fdFrontRod = new b2FixtureDef;
      fdFrontRod->density = 1.0f;
      fdFrontRod->shape = new b2PolygonShape;
      fdFrontRod->shape = &frontRod;
      
      fdFrontRod->filter.groupIndex = -1;
      newBodyFrontRod->CreateFixture(fdFrontRod);
      newBodyFrontRod->SetTransform(newBodyFrontRod->GetPosition(), frontTheta - b2_pi/2);

      b2RevoluteJointDef jointDefFrontRod;
      jointDefFrontRod.bodyA = newBodyFrontRod;
      jointDefFrontRod.bodyB = newBodyFront;
      jointDefFrontRod.localAnchorA.Set(-10.0f * tan(frontTheta),0);
      jointDefFrontRod.localAnchorB.Set(10.0,0);
      jointDefFrontRod.upperAngle = 3*frontTheta;
      jointDefFrontRod.lowerAngle = 3*frontTheta;
      jointDefFrontRod.enableLimit = true;
      jointDefFrontRod.collideConnected = false;
      m_world->CreateJoint(&jointDefFrontRod);

// Front Wheel
/*! \subsection front Front Wheel-Structure
   * 
   * \li \a bdFrontWheel is the bodydefination about the front-wheel body named as \a frontWheel and \a fdFrontWheel is an pointer to 
   * fixture defination of front-wheel. 
   * \li \a frontWheel is in circular shape with radius 10 as described in \a frontWheel defination and is a dynamic body positioned
   *  such that its center coincide with free end of rod \a newBodyFrontRod as described in \a bdFrontWheel.
   * \li Joint- Wheel \a frontWheel and rod \a newBodyFrontRod are joined using revolutejointdef \a jointDefFrontWheel at front-wheel
   *  center.
   * 
   */
      b2CircleShape frontWheel;
      frontWheel.m_radius = 10.0f;
  
      b2BodyDef bdFrontWheel;
      bdFrontWheel.position.Set(-2.0f + (20.0f / cos(frontTheta)), 10.0f);
      bdFrontWheel.type = b2_dynamicBody;
      
      frontWheelBody = m_world->CreateBody(&bdFrontWheel);

      b2FixtureDef *fdFrontWheel = new b2FixtureDef;
      fdFrontWheel->density = 1.0f;
      fdFrontWheel->friction = 20.0f;
      fdFrontWheel->shape = new b2PolygonShape;
      fdFrontWheel->shape = &frontWheel;

      frontWheelBody->CreateFixture(fdFrontWheel);

      b2RevoluteJointDef jointDefFrontWheel;

      jointDefFrontWheel.bodyA = frontWheelBody;
      jointDefFrontWheel.bodyB = newBodyFrontRod;
      jointDefFrontWheel.localAnchorA.Set(0,0);
      jointDefFrontWheel.localAnchorB.Set(10.0f * tan(frontTheta),0);
      jointDefFrontWheel.collideConnected = false;
      m_world->CreateJoint(&jointDefFrontWheel);
      

/// pedal rod
/*! \subsection pedal-rod Pedal Rod
	   * \li Rod named as \a newBody is used to simulate padle motion by fixing rod center with padel-axle center.
	   * \li \a newBody is shaped like a rectangle with dimension (14,0.4) as described in \a pedal and 
	   * positioned at center of padel axle \a body.
	   * \li Joint- Rod \a newBody and pedal-axle \a body are joined using weldjointdef \a pedals at their 
	   * common center and joint collideConnected property is true to ensure same angular velocity of pedal-axle and pedal-rod. 
	   * 
	   */
      b2PolygonShape pedal;
      pedal.SetAsBox(7.0f, 0.2f);
      b2BodyDef bd3;
      bd3.position.Set(-2.0f, 10.0f);
      bd3.type = b2_dynamicBody;
      b2Body* newBody = m_world->CreateBody(&bd3);
      
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.0f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &pedal;
      
      fd2->filter.groupIndex = -1;
      newBody->CreateFixture(fd2);

      b2WeldJointDef pedals;
      pedals.bodyA = body;
      pedals.bodyB = newBody;
      pedals.localAnchorA.Set(0,0);
      pedals.localAnchorB.Set(0,0);
      pedals.collideConnected = true;
      m_world->CreateJoint(&pedals);

/// ankles
/*! \subsection pedal-ankles Pedal-ankles and their joints with padel rod
	   * \li Pedal-structures named as \a ankleRight and \a ankleLeft are used to simulate padle motion by jointing these structure at
	   *  ends of padle rod. 
	   * \li Pedals are shaped like a trapezium with vertices {(-2,0)(2,0)(1,1)(-1,1)} as described in \a poly and 
	   * positioned such that each pedal center coincides with one of the ends of padel-rod \a newBody.
	   * \li Joint- Pedals \a ankleRight and \a ankleLeft are joined with rod \a newBody using revolutejointdef \a ankle1 and \a ankle2
	   *  respectively at the end points of padle-rod. 
	   * 
	   */
      b2RevoluteJointDef ankle1, ankle2;

      b2Body* ankleRight;
      b2Body* ankleLeft;
      //b2Color col = b2Color(0.5,0.5,0.5);
      b2PolygonShape poly;
      b2Vec2 vertices[4];
      vertices[0].Set(-2,0);
      vertices[1].Set(2,0);
      vertices[2].Set(1.0,1.0);
      vertices[3].Set(-1.0,1.0);
      poly.Set(vertices, 4);


      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      wedgefd.filter.groupIndex = -1;
      
      b2BodyDef wedgebdRight;
      wedgebdRight.position.Set(5.0f, 10.0f);
      wedgebdRight.type = b2_dynamicBody;

      
      b2BodyDef wedgebdLeft;
      wedgebdLeft.position.Set(-9.0f, 10.0f);
      wedgebdLeft.type = b2_dynamicBody;

      ankleRight = m_world->CreateBody(&wedgebdRight);
      ankleRight->CreateFixture(&wedgefd);

      ankleLeft = m_world->CreateBody(&wedgebdLeft);
      ankleLeft->CreateFixture(&wedgefd);


      ankle1.bodyA = newBody;
      ankle1.bodyB = ankleRight;
      ankle1.localAnchorA.Set(7.0,0);
      ankle1.localAnchorB.Set(0,0);

      ankle1.collideConnected = false;
      m_world->CreateJoint(&ankle1);
      
      ankle2.bodyA = newBody;
      ankle2.bodyB = ankleLeft;
      ankle2.localAnchorA.Set(-7.0,0);
      ankle2.localAnchorB.Set(0,0);
   
      ankle2.collideConnected = false;
      m_world->CreateJoint(&ankle2);
      
     
/// Hip Joint Disc
/*! \subsection hip-joint-disc Hip Joint Disc
	   * \li Hip-structure named as \a bodyHip used to simulate biker hip combined with cycle seat part motion.
	   * \li Hip is shaped in circular shape with radius 2 as described in \a shapeHip and is positioned at (-2,25) and is assigned 
	   * density 100 in b2FixtureDef pointer \a fdHip.
	   * \li Joint- Hip \a bodyHip is joined with padel-axle \a body using distancejointdef \a jointDefHip to provide stability to
	   *  biker.This distance joint is used to create suspension by assigning frequencyHz as 2 and dampingRatio 0.6 in \a jointdefHip. 
	   * 
	   */
    /*
      b2Vec2 hip_vertices[4];
      hip_vertices[0].Set(-2,0);
      hip_vertices[1].Set(2,0);
      hip_vertices[2].Set(1.0,1.0);
      hip_vertices[3].Set(-1.0,1.0);
      hip_shapeHip.Set(hip_vertices, 4);
      */
      
      b2CircleShape shapeHip;
      shapeHip.m_radius = 2.0f;

      b2BodyDef bdHip;
      bdHip.position.Set(-2.0f, 25.0f);
      bdHip.type = b2_dynamicBody;

      b2Body* bodyHip = m_world->CreateBody(&bdHip);
      b2FixtureDef *fdHip = new b2FixtureDef;
      fdHip->density = 100.f;
      fdHip->shape = new b2PolygonShape;
      fdHip->shape = &shapeHip;
      fdHip->filter.groupIndex = -1;
      

      bodyHip->CreateFixture(fdHip);

      b2DistanceJointDef jointDefHip;
      jointDefHip.bodyA = bodyHip;
      jointDefHip.bodyB = body;

      jointDefHip.localAnchorA.Set(0,-15.0);
      jointDefHip.localAnchorB.Set(0,0);
      jointDefHip.collideConnected = false;
      jointDefHip.frequencyHz = 2.0f;
      jointDefHip.dampingRatio = 0.6f;
      m_world->CreateJoint(&jointDefHip);
      /// Right Thigh
/*! \subsection right-thigh Right Thigh
	   * \li Right Thigh-structure named as \a newBodyThighRight used to simulate biker right thigh and thigh is hinged to hip-center.
	   * \li \a newBodyThighRight is shaped in rectangular shape with dimension (12,2) as described in \a thigh and is positioned such 
	   * that its one end coincides with the center of hip \a hipBody as described in \a bd3ThighRight.
	   * \li To ensure proper posture of thigh , thigh structure is rotated \a alpha angle clockwise using SetTransform.
	   * \li Joint- Thigh \a newBodyThighRight and Hip \a bodyHip are joined using revolutejointdef \a jointThighRight at the center of
	   * hip \a bodyHip.
	   * 
	   */
      b2PolygonShape thigh;
      thigh.SetAsBox(6.0f, 1.0f);

      float alpha = 0.324456;

      b2BodyDef bd3ThighRight;
      bd3ThighRight.position.Set(-2.0f + (cos(alpha)*6.0f), 25.0f - (sin(alpha)*6.0f));
      bd3ThighRight.type = b2_dynamicBody;
      b2Body* newBodyThighRight = m_world->CreateBody(&bd3ThighRight);

      b2FixtureDef *fd2ThighRight = new b2FixtureDef;
      fd2ThighRight->density = 1.0f;
      fd2ThighRight->shape = new b2PolygonShape;
      fd2ThighRight->shape = &thigh;
      

      fd2ThighRight->filter.groupIndex = -1;
      newBodyThighRight->CreateFixture(fd2ThighRight);
 
      newBodyThighRight->SetTransform(newBodyThighRight->GetPosition(), -1*alpha);

      b2RevoluteJointDef jointThighRight;

      jointThighRight.bodyA = newBodyThighRight;
      jointThighRight.bodyB = bodyHip;
      jointThighRight.localAnchorA.Set(-6.0, 0.0);
      jointThighRight.localAnchorB.Set(0,0);
      jointThighRight.collideConnected = false;
      m_world->CreateJoint(&jointThighRight);



/// Right Leg
/*! \subsection right-leg Right Leg
	   * \li Right Leg-structure named as \a newBodyLegRight is used to simulate biker right leg and leg is hinged to free end of 
	   * right thigh \a newBodyThighRight.
	   * \li \a newBodyLegRight is shaped in rectangular shape with dimension (12,1) as described in \a leg and is positioned such 
	   * that its one end coincides with the free end of right-thigh \a newBodyThighRight as described in \a bd3LegRight.
	   * \li To ensure proper positioning of leg such that it touches right side padel appropriate angle \a beta is choosen and
	   *  right leg structure is rotated \a beta angle clockwise using SetTransform.
	   * \li Joint- Leg \a newBodyLegRight and Thigh \a newBodyThighRight are joined using revolutejointdef \a jointLegRight at their 
	   * common overlapping point.
	   * 
	   */      
      b2PolygonShape leg;
      leg.SetAsBox(6.0f, 0.5f);

      float beta = 1.19771;

      b2BodyDef bd3LegRight;
      bd3LegRight.position.Set(5.0f + (cos(beta) * 6.0f), 10.0f + (sin(beta) * 6));
      bd3LegRight.type = b2_dynamicBody;
      b2Body* newBodyLegRight = m_world->CreateBody(&bd3LegRight);
      
      b2FixtureDef *fd2LegRight = new b2FixtureDef;
      fd2LegRight->density = 1.0f;
      fd2LegRight->shape = new b2PolygonShape;
      fd2LegRight->shape = &leg;
      

      fd2LegRight->filter.groupIndex = -1;
      newBodyLegRight->CreateFixture(fd2LegRight);

      newBodyLegRight->SetTransform(newBodyLegRight->GetPosition(), beta);

      b2RevoluteJointDef jointLegRight;

      jointLegRight.bodyA = newBodyLegRight;
      jointLegRight.bodyB = newBodyThighRight;
      jointLegRight.localAnchorA.Set(6.0, 0);
      jointLegRight.localAnchorB.Set(6.0,0);
      jointLegRight.collideConnected = false;
      m_world->CreateJoint(&jointLegRight);

/// Left Thigh
/*! \subsection left-thigh Left Thigh
	   * \li Left Thigh-structure named as \a newBodyThighLeft used to simulate biker left thigh and thigh is hinged to hip-center.
	   * \li \a newBodyThighLeft is shaped in rectangular shape with dimension (12,2) as described in \a thigh and is positioned such 
	   * that its one end coincides with the center of hip \a hipBody as described in \a bd3ThighLeft.
	   * \li To ensure proper posture of thigh , thigh structure is rotated \a beta angle clockwise using SetTransform.
	   * \li Joint- Left-Thigh \a newBodyThighLeft and Hip \a bodyHip are joined using revolutejointdef \a jointThighLeft at the center of
	   * hip \a bodyHip.
	   * 
	   */
      b2BodyDef bd3ThighLeft;
      bd3ThighLeft.position.Set(-2.0f + (sin(beta) * 3.0f), 25.0f - (sin(beta)*6.0f));

      bd3ThighLeft.type = b2_dynamicBody;
      b2Body* newBodyThighLeft = m_world->CreateBody(&bd3ThighLeft);
      
      b2FixtureDef *fd2ThighLeft = new b2FixtureDef;
      fd2ThighLeft->density = 1.0f;
      fd2ThighLeft->shape = new b2PolygonShape;
      fd2ThighLeft->shape = &thigh;
      

      fd2ThighLeft->filter.groupIndex = -1;
      newBodyThighLeft->CreateFixture(fd2ThighLeft);
      newBodyThighLeft->SetTransform(newBodyThighLeft->GetPosition(), -1*beta);

      b2RevoluteJointDef jointThighLeft;

      jointThighLeft.bodyA = newBodyThighLeft;
      jointThighLeft.bodyB = bodyHip;
      jointThighLeft.localAnchorA.Set(-6.0, 0);
      jointThighLeft.localAnchorB.Set(0,0);
      jointThighLeft.collideConnected = false;
      m_world->CreateJoint(&jointThighLeft);


/// Left Leg
/*! \subsection left-leg Left Leg
	   * \li Left Leg-structure named as \a newBodyLegRight is used to simulate biker left leg and leg is hinged to free end of 
	   * left thigh \a newBodyThighLeft.
	   * \li \a newBodyLegLeft is shaped in rectangular shape with dimension (12,1) as described in \a leg and is positioned such 
	   * that its one end coincides with the free end of left-thigh \a newBodyThighLeft as described in \a bd3LegLeft.
	   * \li To ensure proper positioning of leg such that it touches left side pedal left leg structure is rotated \a alpha angle 
	   * anti-clockwise using SetTransform.
	   * \li Joint- Leg \a newBodyLegLeft and Left-Thigh \a newBodyThighLeft are joined using revolutejointdef \a jointLegLeft at 
	   * their common overlapping point.
	   * 
	   */      
      b2BodyDef bd3LegLeft;
      bd3LegLeft.position.Set(-9.0f + (cos(alpha) * 6.0f), 10.0f + (sin(alpha) * 6.0f));

      bd3LegLeft.type = b2_dynamicBody;
      b2Body* newBodyLegLeft = m_world->CreateBody(&bd3LegLeft);
      
      b2FixtureDef *fd2LegLeft = new b2FixtureDef;
      fd2LegLeft->density = 1.0f;
      fd2LegLeft->shape = new b2PolygonShape;
      fd2LegLeft->shape = &leg;
      
      fd2LegLeft->filter.groupIndex = -1;
      newBodyLegLeft->CreateFixture(fd2LegLeft);

      newBodyLegLeft->SetTransform(newBodyLegLeft->GetPosition(), alpha);

      b2RevoluteJointDef jointLegLeft;

      jointLegLeft.bodyA = newBodyLegLeft;
      jointLegLeft.bodyB = newBodyThighLeft;
      jointLegLeft.localAnchorA.Set(6.0, 0);
      jointLegLeft.localAnchorB.Set(6.0, 0);
      jointLegLeft.collideConnected = false;
      m_world->CreateJoint(&jointLegLeft);
      

  /// Joints to join legs and ankle
/*! \subsection ankle-leg-joint Ankle(Pedal) and leg joint 
       * \li This part creates 2 joints for joining left and right ankles(pedal) with left and right legs respectively. 
       * \li Joint1- Right-leg \a newBodyLegRight and right-ankle \a ankleRight are joined using revolutejointdef \a rightAnkle at 
	   * center of right ankle(padle).In \a rightAnkle limits are enabled with angle ranging from (b2_pi/12 - 3*b2_pi/4) to 
	   * (-1*b2_pi/12 - 3*b2_pi/4) to simulate ankle motion about joint.
	   * \li Joint1- Left-leg \a newBodyLegLeft and left-ankle \a ankleLeft are joined using revolutejointdef \a leftAnkle at 
	   * center of left ankle(padle).In \a leftAnkle limits are enabled with angle ranging from (b2_pi/12 - 3*b2_pi/4) to 
	   * (-1*b2_pi/12 - 3*b2_pi/4) to simulate ankle motion about joint.
	   *
	   */
      b2RevoluteJointDef leftAnkle, rightAnkle;
      rightAnkle.bodyA = ankleRight;
      rightAnkle.bodyB = newBodyLegRight;
      rightAnkle.localAnchorA.Set(0, 0);
      rightAnkle.localAnchorB.Set(-6.0f, 0);
      rightAnkle.collideConnected = false;
      rightAnkle.upperAngle = b2_pi/12 - 3*b2_pi/4;
      rightAnkle.lowerAngle = -1*b2_pi/12 - 3*b2_pi/4;
      rightAnkle.enableLimit = true;
      m_world->CreateJoint(&rightAnkle);

      leftAnkle.bodyA = ankleLeft;
      leftAnkle.bodyB = newBodyLegLeft;
      leftAnkle.localAnchorA.Set(0, 0);
      leftAnkle.localAnchorB.Set(-6.0f, 0);
      leftAnkle.collideConnected = false;
      leftAnkle.upperAngle = b2_pi/12 - 3*b2_pi/4;
      leftAnkle.lowerAngle = -1*b2_pi/12 - 3*b2_pi/4;
      leftAnkle.enableLimit = true;
      m_world->CreateJoint(&leftAnkle);


      
      /// Two boxes and a circle for the head

      /// The box
/// The main body of the driver
/*! \subsection driver-body Driver Main Body Parts
	  * \li This part creates upper body part structure of driver named as \a stomachBody.
      * \li \a stomach is shaped in rectangular shape with dimension (12,5) as described in \a stomach and is positioned such 
	  * that mid point of one of the shorter edge coincides wiht the center of hip \a bodyHip as described in \a bdStomach. 
	  * \li To simulate proper posture of driver, upper part \a stomach is rotated pi/3 angle anti-clockwise using SetTransform.
	  * \li Joint - Body-part \a stomach and hip \a bodyHip are joined using revolutejointdef \a stomachJoint at center of \a bodyHip.
	  * 
      */ 

      b2PolygonShape stomach;
      stomach.SetAsBox(6.0f, 2.5f);

      b2BodyDef bdStomach;
      bdStomach.position.Set(1.0f, 25.0f + (sin(b2_pi/3) * 6.0f));
      bdStomach.type = b2_dynamicBody;
      b2Body* stomachBody = m_world->CreateBody(&bdStomach);
      
      b2FixtureDef *fdStomach = new b2FixtureDef;
      fdStomach->density = 1.f;
      fdStomach->shape = new b2PolygonShape;
      fdStomach->shape = &stomach;
      
      fdStomach->filter.groupIndex = -1;
      stomachBody->CreateFixture(fdStomach);
      stomachBody->SetTransform(stomachBody->GetPosition(), b2_pi/3);
      
      b2RevoluteJointDef stomachJoint;
      
      stomachJoint.bodyA = stomachBody;
      stomachJoint.bodyB = bodyHip;
      stomachJoint.localAnchorA.Set(-6.0, 0);
      stomachJoint.localAnchorB.Set(0,0);
      
      stomachJoint.upperAngle = b2_pi/24 - b2_pi/3;
      stomachJoint.lowerAngle = -1*b2_pi/24 - b2_pi/3;
      stomachJoint.enableLimit = true;
      
      stomachJoint.collideConnected = false;
      m_world->CreateJoint(&stomachJoint);
      
/// Head
/*! \subsection driver-body Driver Main Body Parts
	  * \li This part creates head structure of driver named as \a head.
      * \li \a head is in circular shape with radius 4 as described in \a shapeHead and is positioned such 
	  * that mid point of one of the shorter edge coincides wiht the center of hip \a bodyHip as described in \a bdStomach. 
	  * \li To simulate proper posture of driver, upper part \a stomach is rotated pi/3 angle anti-clockwise using SetTransform.
	  * \li Joint - Body-part \a stomach and hip \a bodyHip are joined using revolutejointdef \a stomachJoint at center of \a bodyHip.
	  * 
      */ 


      b2CircleShape shapeHead;
      shapeHead.m_radius = 4.0f;
  
      b2BodyDef bdHead;
      bdHead.position.Set(-2.0f + 14.0f/2, 25.0f + (sin(b2_pi/3) * 14.0f));
      bdHead.type = b2_dynamicBody;
      b2Body* head = m_world->CreateBody(&bdHead);

      b2FixtureDef *fdHead = new b2FixtureDef;
      fdHead->density = 1.0f;
      fdHead->shape = new b2PolygonShape;
      fdHead->shape = &shape;

      head->CreateFixture(fdHead);
      
      b2WeldJointDef neck;

      neck.bodyA = head;
      neck.bodyB = stomachBody;
      neck.localAnchorA.Set(-2.0,0);
      neck.localAnchorB.Set(6.0,0);
      neck.collideConnected = false;
      m_world->CreateJoint(&neck);

/// Rod from hip to front part of cycle
/*! \subsection supportrod3 Hip to front part joining rod
	  * \li This part creates a rod named as \a newHipRod to support driver hip by connecting it to front part of cycle.
      * \li \a newHipRoad is in rectangular shape with dimension (19,0.5) as described in \a hipRod and is positioned such 
	  * that one end coincides with the center of hip and another end coincides with the forward joint of other 2 supporting rods.
	  * \li Rod \a newHipRod is rotated 0.3 radian angle clockwise using SetTransform.
	  * \li Joint1 - Rod \a newHipRod and hip \a bodyHip are joined using revolutejointdef \a jointHipFront at center of \a bodyHip.
	  * \li Joint2 - Rod \a newHipRod and rod \a newBodyFrontRod are joined using revolutejointdef \a jointFrontRod at their common 
	  * overlapping point.
	  * 
      */ 
      b2PolygonShape hipRod;
      hipRod.SetAsBox(9.5f, 0.25f);

      b2BodyDef bdHipRod;
      bdHipRod.position.Set(6.6f, 22.3f);

      bdHipRod.type = b2_dynamicBody;
      b2Body* newHipRod = m_world->CreateBody(&bdHipRod);
      
      b2FixtureDef *fd2Hip = new b2FixtureDef;
      fd2Hip->density = 1.0f;
      fd2Hip->shape = new b2PolygonShape;
      fd2Hip->shape = &hipRod;
      
      fd2Hip->filter.groupIndex = -1;
      newHipRod->CreateFixture(fd2Hip);

      newHipRod->SetTransform(newHipRod->GetPosition(), -0.30);

      b2RevoluteJointDef jointHipFront, jointFrontRod;

      jointHipFront.bodyA = newHipRod;
      jointHipFront.bodyB = bodyHip;
      jointHipFront.localAnchorA.Set(-9.5, 0);
      jointHipFront.localAnchorB.Set(0.0, 0);
      jointHipFront.collideConnected = false;
      m_world->CreateJoint(&jointHipFront);

      jointFrontRod.bodyA = newHipRod;
      jointFrontRod.bodyB = newBodyFrontRod;
      jointFrontRod.localAnchorA.Set(9.5, 0);
      jointFrontRod.localAnchorB.Set(-10.0f * tan(frontTheta), 0);
      jointFrontRod.collideConnected = false;
      m_world->CreateJoint(&jointFrontRod);
      
/// The handle joining hand and front rod 
/*! \subsection handle Handle Joining Rod
	  * \li This part creates a rod named as \a newHandleRod to simulate handle part of the cycle.
      * \li \a newHandleRoad is in rectangular shape with dimension (0.5,8) as described in \a handleRod and is positioned such 
	  * that one end coincides with front cycle joint of bodies \a newHipRod and other supporting rods.
	  * \li Joint - Rod \a newHandleRod and hip \a bodyHip are joined using revolutejointdef \a jointHandle at their common 
	  * overlapping point.
	  * 
      */ 
      b2PolygonShape handleRod;
      handleRod.SetAsBox(0.25f, 4.0f);

      b2BodyDef bdHandleRod;
      bdHandleRod.position.Set(-2.0f + (cos(frontTheta) * 20.0f), 14.0f + (sin(frontTheta) * 20.0f));

      bdHandleRod.type = b2_dynamicBody;
      b2Body* newHandleRod = m_world->CreateBody(&bdHandleRod);
      
      b2FixtureDef *fd2Handle = new b2FixtureDef;
      fd2Handle->density = 1.0f;
      fd2Handle->shape = new b2PolygonShape;
      fd2Handle->shape = &handleRod;
      
      newHandleRod->CreateFixture(fd2Handle);

      b2RevoluteJointDef jointHandle;

      jointHandle.bodyA = newHandleRod;
      jointHandle.bodyB = newBodyFront;
      jointHandle.localAnchorA.Set(0, -4.0);
      jointHandle.localAnchorB.Set(10.0,0);
      jointHandle.collideConnected = false;
      jointHandle.upperAngle = frontTheta;
      jointHandle.lowerAngle = frontTheta;
      jointHandle.enableLimit = true;
      m_world->CreateJoint(&jointHandle);

/// The hand of human
/*! \subsection hand Driver hand
	  * \li This part simulates driver hand by creating a rod named as \a newHandRod.
      * \li \a newHandRoad is in rectangular shape with dimension (12,0.8) as described in \a handRod and is positioned such 
	  * that one end coincides with open end of handleRod \a newHandleRod and another end coincides with driverbody.
	  * \li Joint - Rod \a newHandRod and rod \a newHandleRod are joined using revolutejointdef \a jointHand at their common 
	  * overlapping point.
	  * 
      */ 
      //bdStomach.position.Set(1.0f, 25.0f + (sin(b2_pi/3) * 6.0f));
      
      b2PolygonShape handRod;
      handRod.SetAsBox(6.0f, 0.4f);

      b2BodyDef bdHandRod;
      bdHandRod.position.Set((cos(frontTheta) * 10.0f), 30.0f);

      bdHandRod.type = b2_dynamicBody;
      b2Body* newHandRod = m_world->CreateBody(&bdHandRod);
      
      b2FixtureDef *fd2Hand = new b2FixtureDef;
      fd2Hand->density = 1.0f;
      fd2Hand->shape = new b2PolygonShape;
      fd2Hand->shape = &handRod;
      fd2Hand->filter.groupIndex = -1;
      
      newHandRod->CreateFixture(fd2Hand);
      newHandRod->SetTransform(newHandRod->GetPosition(), -1 * frontTheta);

      b2RevoluteJointDef jointHand;

      jointHand.bodyA = newHandleRod;
      jointHand.bodyB = newHandRod;
      jointHand.localAnchorA.Set(0, 4.0);
      jointHand.localAnchorB.Set(6.0,0);
      jointHand.collideConnected = false;
      jointHand.upperAngle = b2_pi/6 - frontTheta;
      jointHand.lowerAngle = b2_pi/24 - frontTheta;
      jointHand.enableLimit = true;
      m_world->CreateJoint(&jointHand);

/// Joint Connecting Hand to Shoulder (stomachBody)
/*! \subsection joint Hand-Body Joint
	  * \li This part creates a distancejoint between handrod and main body part of driver.
	  * \li Joint positions are choosen such that joinst are neraby the overlapping point of \a stomachBody and \a newHandRod.  
      */ 
      b2DistanceJointDef jointShoulder;

      jointShoulder.bodyA = stomachBody;
      jointShoulder.bodyB = newHandRod;
      jointShoulder.localAnchorA.Set(4.0, -2.0);
      jointShoulder.localAnchorB.Set(-6.0, 0);
      jointShoulder.collideConnected = false;

      /*
      jointShoulder.upperAngle = b2_pi/12 - frontTheta;
      jointShoulder.lowerAngle = -1*b2_pi/24 - frontTheta;
      jointShoulder.enableLimit = true;
      */

      m_world->CreateJoint(&jointShoulder);
      

/// The chain running between the rear wheel and the pedal
/*!\subsection chain Cycle-Chain
 * \li Most important and complex part of cycle is chain as its trajectory is both circular and linear between the pedal axle and rear
 * axle and its orientation changes instantaneously while changing gear.
 *
 * \li Chain is created by joining small rectangular shaped element of dimension (1.2,0.25) as described in \a shape.Revolutejoint 
 * named as \a jd are used to join these elements.
 *
 * \li While joining,elements are positioned such that their is no gap between previous element and new element.Joints are done at the
 * the end point to ensure that chain remains rigid and moves with minimum slack and distortion. 
 *
 * \li Element fixture \a fd assigns density and friction values as 20 and 40 respectively.Here friction value is high as friction
 * between axles and chain elements is the main driving force for whole cycle simulation.
 *
 * \li Element \a element is assigned proper mask and category bits such that it collides only with padel and gear-axles and does not 
 * collide with padel rod, rear wheel and driver legs.
 * \li Initially rod is simply created in box kind structure and stuck between small pullies and axles. Number of elements to be
 *  choosen are taken appropriately such that chain remains tight and minimum slack is generated.   
 * \li Here each for loop creates a linear part of chain.In each iteration of for loop a unit of \a element is added to chain.
 * \li As simulation starts chain structure automatically takes appropriate posture by colliding with axles and small pullies.
 * \li Chain as driving force - Angular velocity given to pedal axle initiates motion of chain and high coefficient of friction 
 * further stops the relative motion between chain and axles.Now as rear axles are hinged to rear wheel thus rear wheel obtains some
 * angular velocity. Now the friction between ground and wheel ensures that contact point remains at rest and a translational motion
 * is initiated for whole cycle.
 *  
 */
  {
      b2PolygonShape shape;
      shape.SetAsBox(0.6f, 0.125f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 30.0f;

      //fd.filter.groupIndex = -1;

      fd.filter.categoryBits = 0x0004;
      fd.filter.maskBits = 0x0002;

      b2RevoluteJointDef jd;
      jd.collideConnected = false;

      float32 y = 11.0f, x = -18.0f;
      float32 itr = 17.0f;

      b2Body* prevBody = b1;
      b2Body* startBody;

      float32 i = 0;
      for (i = 0; i < itr ; ++i)
      {
        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(x + 0.5f + i, y);
        b2Body* element = m_world->CreateBody(&bd);
        element->CreateFixture(&fd);

        if (i == 0){
          startBody = element;
        }

        if (i != 0){

            b2Vec2 anchor(float32(x + i), y);
            jd.Initialize(prevBody, element, anchor);
            jd.collideConnected = false;
            m_world->CreateJoint(&jd);
        }

        prevBody = element;
      }

      x = x + itr;
      for (i = 0; i < 8; ++i)
      {
        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(x, y  + 0.5f - (i+1));
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);
        body->SetTransform(body->GetPosition(), b2_pi/2);

            b2Vec2 anchor(x, y - i);
            jd.Initialize(prevBody, body, anchor);
            jd.collideConnected = false;
            m_world->CreateJoint(&jd);

        prevBody = body;
      }

      y = y - 8.0f;
      for (i = 0; i < 13; ++i)
      {
        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(x  + 0.5f - (i+1), y);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);

            b2Vec2 anchor(x - i, y);
            jd.Initialize(prevBody, body, anchor);
            jd.collideConnected = false;
            m_world->CreateJoint(&jd);

        prevBody = body;
      }

      x = x - 13.0f;
      for (i = 0; i < 3; ++i)
      {
        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(x, y  + 0.5f + i);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);
        body->SetTransform(body->GetPosition(), b2_pi/2);

            b2Vec2 anchor(x, y + i);
            jd.Initialize(prevBody, body, anchor);
            jd.collideConnected = false;
            m_world->CreateJoint(&jd);

        prevBody = body;
      }

      y = y + 3.0f;
      for (i = 0; i < 3; ++i)
      {
        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(x  + 0.5f + i, y);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);

            b2Vec2 anchor(x + i, y);
            jd.Initialize(prevBody, body, anchor);
            jd.collideConnected = false;
            m_world->CreateJoint(&jd);

        prevBody = body;
      }

      x = x + 3.0f;
      for (i = 0; i < 3; ++i)
      {
        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(x, y  + 0.5f + i);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);
        body->SetTransform(body->GetPosition(), b2_pi/2);

            b2Vec2 anchor(x, y + i);
            jd.Initialize(prevBody, body, anchor);
            jd.collideConnected = false;
            m_world->CreateJoint(&jd);

        prevBody = body;
      }

      y = y + 3.0f;
      for (i = 0; i < 7; ++i)
      {
        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(x  + 0.5f - (i+1), y);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);

            b2Vec2 anchor(x - i, y);
            jd.Initialize(prevBody, body, anchor);
            jd.collideConnected = false;
            m_world->CreateJoint(&jd);

        prevBody = body;
      }

      x = x - 7.0f;
      for (i = 0; i < 2; ++i)
      {
        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(x, y  + 0.5f + i);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);
        body->SetTransform(body->GetPosition(), b2_pi/2);

            b2Vec2 anchor(x, y + i);
            jd.Initialize(prevBody, body, anchor);
            jd.collideConnected = false;
            m_world->CreateJoint(&jd);

        prevBody = body;
      }

      y += 2.0f;
      b2Vec2 lastAnchor(x, y);
      jd.Initialize(prevBody, startBody, lastAnchor);
      jd.collideConnected = false;
      m_world->CreateJoint(&jd);
      
    }
    
}
/*!\section KEYBOARD EVENTS
 */

  void dominos_t::keyboard(unsigned char key){
    switch(key){
/*!\subsection KeyBoard event 'd'
 * \li Keyboard key \a d is used to decrease gear by increasing \a gearIndex variable.
 *
 * \li It destroys the current gearaxle using \a DestroyBody memberfunction and increase gearIndex by 1.
 */      
      case 'd':
        if (gearIndex == 1){
          m_world->DestroyBody(gear1);
          gear1 = NULL;
          gearIndex++;
        }

        else if (gearIndex == 2){
          m_world->DestroyBody(gear2);
          gear2 = NULL;
          gearIndex++;
        }
      break;

      case 'i':
 /*!\subsection KeyBoard event 'i'
		* \li Keyboard key \a i is used to increase rear axle-radius. 
		* \li In case of 'i' press event a whole new gear structure is created with increased axle-radius. Body named as \a gear2 or
		* \a gear1 are created depending upon the present gearIndex value.
		* \li These bodies are positioned at the center of rear wheel as described in \a bd.Category and mask bits are assigned
		* such that new axle collides with the chain structure and chain gets reshaped with a larger radius axle.   
		*/ 
        if (gearIndex == 3){
          {
              gear3->SetAngularVelocity(0.0f);
              b2BodyDef bdGear;
              bdGear.position.Set(gear3->GetPosition().x, gear3->GetPosition().y);
              bdGear.type = b2_dynamicBody;
              b2CircleShape shapeGear2;
              shapeGear2.m_radius = 4.0f - 1.0f;
              
              gear2 = m_world->CreateBody(&bdGear);

              b2FixtureDef *fdGear2 = new b2FixtureDef;
              {
                fdGear2->density = 10.0f;
                fdGear2->shape = new b2PolygonShape;
                fdGear2->shape = &shapeGear2;

                fdGear2->filter.categoryBits = 0x0002;
                fdGear2->filter.maskBits = 0x0004;
              }
              gear2->CreateFixture(fdGear2);

              b2WeldJointDef jointDefGear2;
              jointDefGear2.bodyA = gear2;
              jointDefGear2.bodyB = body2;

              jointDefGear2.localAnchorA.Set(0,0);
              jointDefGear2.localAnchorB.Set(0,0);
              jointDefGear2.collideConnected = false;
              m_world->CreateJoint(&jointDefGear2);
          }
          gearIndex--;
        }

        else if (gearIndex == 2){
          {
            gear2->SetAngularVelocity(0.0f);
            b2BodyDef bdGear;
            bdGear.position.Set(gear2->GetPosition().x, gear2->GetPosition().y);
            bdGear.type = b2_dynamicBody;

            b2CircleShape shapeGear1;
            shapeGear1.m_radius = 4.0f;
        
            gear1 = m_world->CreateBody(&bdGear);

            b2FixtureDef *fdGear1 = new b2FixtureDef;
            {
              fdGear1->density = 10.0f;
              fdGear1->shape = new b2PolygonShape;
              fdGear1->shape = &shapeGear1;

              fdGear1->filter.categoryBits = 0x0002;
              fdGear1->filter.maskBits = 0x0004;
            }
            gear1->CreateFixture(fdGear1);


            b2WeldJointDef jointDefGear1;
            jointDefGear1.bodyA = gear1;
            jointDefGear1.bodyB = body2;

            jointDefGear1.localAnchorA.Set(0,0);
            jointDefGear1.localAnchorB.Set(0,0);
            jointDefGear1.collideConnected = false;
            m_world->CreateJoint(&jointDefGear1);
          }
          gearIndex--;
        }
      break;

/*!\subsection KeyBoard event 'q'
 * \li Keyboard key \a d is used for the forward motion of the cycle.
 * \li In this part angular velocity of pedal axle is increased in clockwise direction to initiate forward motion of cycle.
 * \li An upper limit of 8 is kept for angular velocity to ensure that chain does not get distroted with high angular motion.
 * 
 */   
    case 'q':
      if (body->GetAngularVelocity() > -8.0f){
        body->SetAngularVelocity(body->GetAngularVelocity() - 1.0f);
      }
      break;

/*!\subsection KeyBoard event 'w'
 * \li Keyboard key \a w is used for the backward motion of the cycle.
 * \li In this part angular velocity of pedal axle is increased in anti-clockwise direction to initiate backward motion of cycle.
 * \li An upper limit of 8 is kept for angular velocity to ensure that chain does not get distroted with high angular motion.
 * 
 */  

    case 'w':
      if (body->GetAngularVelocity() < 8.0f){
        body->SetAngularVelocity(body->GetAngularVelocity() + 1.0f);
      }
      break;

    case 'b':
      if (frontWheelBody->GetAngularVelocity() > 0){
        frontWheelBody->SetAngularVelocity(frontWheelBody->GetAngularVelocity() - 0.5f);
      }
      else{
       frontWheelBody->SetAngularVelocity(frontWheelBody->GetAngularVelocity() + 0.5f); 
      }
      break;

    }
  }


  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
