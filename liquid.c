/**
    This file is a modification from the drawstuff example taken from
    Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.

    LIQUID-BALLS:

    Simulate water spheres being pour into a box. Write out frame step info
    with each sphere positions in numbered frame files in data/%04d.dat
    Optionaly shows simulation graphics.

    Author: Rodrigo Setti <rodrigosetti@gmail.com>
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ode/ode.h>

/******************************************************************************
    SETTINGS SECTION: Probably this is the only section user needs to edit, 
    but please feel free to modify this whole file at will(sorry about the mess)
*******************************************************************************/

/* Uncomment to show graphics */
// #define GRAPHICS

/* If graphics are enabled, uncomment to show boxes */
// #define DRAW_BOXES

/* Water sphere settings */
#define NUM 2000        /* number of water spheres */
#define RADIUS (0.05f)  /* water sphere radius */

/* Box dimensions(where the water is pour into) */
#define BOXW (6.0f)
#define BOXH (6.0f)
#define BOXD (0.1f)

/* Simulation step size(in seconds) */
#define STEP 0.05f

/******************************************************************************/

#ifdef GRAPHICS
#include "drawstuff/drawstuff.h"

#ifndef DRAWSTUFF_TEXTURE_PATH
#define DRAWSTUFF_TEXTURE_PATH "drawstuff/textures/"
#endif

/* select correct drawing functions */

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

#endif

#define MASS (1.0f)     /* mass of a box */
#define C 10.0f

/* dynamics and collision objects */

static dWorldID world;
static dSpaceID space;
static dBodyID body[NUM];
static dJointGroupID contactgroup;
static dGeomID sphere[NUM];
static int cNum;
static dMass m;
static int doneCreation;

static dGeomID g1, g2, g3, g4, g5;

static const dReal s1[3] = { BOXD, BOXW, BOXH };
static const dReal s2[3] = { BOXD, BOXW, BOXH };
static const dReal s3[3] = { BOXW, BOXD, BOXH };
static const dReal s4[3] = { BOXW, BOXD, BOXH };
static const dReal s5[3] = { BOXW, BOXW, BOXD };

/* 
 * this is called by dSpaceCollide when two objects in space are
 * potentially colliding.
 */
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
    /* exit without doing anything if the two bodies are connected by a joint */
    dBodyID b1,b2;
    dContact contact;

    /* If objects are both not spheres, do not collide */
    if (dGeomGetClass(o1) != dSphereClass && dGeomGetClass(o2) != dSphereClass) return;

    b1 = dGeomGetBody(o1);
    b2 = dGeomGetBody(o2);

    /* If objects exists */
    if (b1 && b2)
    {
        /* Get positions */
        dReal *p1 = (dReal*)dBodyGetPosition(b1);
        dReal *p2 = (dReal*)dBodyGetPosition(b2);

        /* Spheres don't collide if they're up box */
        if (p1[2] > 1.5*BOXH || p2[2] > 1.5*BOXH) return;
    }

    /* Set collision parameters */
    contact.surface.mode =
    (dGeomGetClass(o1) == dGeomGetClass(o2) == dSphereClass?
        dContactBounce : 0);
    contact.surface.mu = .1;
    contact.surface.mu2 = 0;
    contact.surface.bounce = .3;

    /* If they boudings overlap: collision */
    if (dCollide (o1,o2, 1, &contact.geom, sizeof(dContactGeom)))
    {

        /* If first object is sphere: add force normal to contact */
        if (dGeomGetClass(o1) == dSphereClass)
        {
            dBodyAddForce(b1, -C*contact.geom.normal[0]*contact.geom.depth,
                -C*contact.geom.normal[1]*contact.geom.depth,
                -C*contact.geom.normal[2]*contact.geom.depth);
        }
        /* If second object is sphere: add force normal to contact */
        if (dGeomGetClass(o2) == dSphereClass)
        {
            dBodyAddForce(b2, C*contact.geom.normal[0]*contact.geom.depth,
                C*contact.geom.normal[1]*contact.geom.depth,
                C*contact.geom.normal[2]*contact.geom.depth);
        }

        /* Create a joint between the two objects and attach to world */
        dJointID c = dJointCreateContact (world, contactgroup,&contact);
        dJointAttach (c,b1,b2);
    }
}


/**
    Start simulation: set viewpoint if graphics
    and init ODE
*/
static void start()
{
    #ifdef GRAPHICS
    static float xyz[3] = {2.1640f,-1.3079f,1.7600f};
    static float hpr[3] = {125.5000f,-17.0000f,0.0000f};

    dsSetViewpoint (xyz,hpr);
    #endif
    dAllocateODEDataForThread(dAllocateMaskAll);
}


/** 
    Main simulation loop
*/
static void simLoop (int pause)
{
    static unsigned int iter = 1;
    dReal *bodypos;
    int i;

    if (!pause) {
        /* next iteration */
        iter++;

        if (cNum < NUM)
        {
            /* Create body and atach to world */
            if (!doneCreation)
            {
                body[cNum] = dBodyCreate(world);
                dMassSetBox(&m, 1, RADIUS, RADIUS, RADIUS);
                dMassAdjust(&m, MASS);
                dBodySetMass(body[cNum], &m);
                sphere[cNum] = dCreateSphere(space, RADIUS +
                                             (RADIUS*(rand() % 1000)/2000.0f));
                dGeomSetBody(sphere[cNum],body[cNum]);
            }

            /* Set water sphere initial position: source */
            dBodySetPosition(body[cNum], RADIUS*(rand() % 100)/100.0f,
                             RADIUS*(rand() % 200)/100.0f,
                             10.0f);

            /* Sets water sphere initial velocity: spread */
            dBodySetLinearVel(body[cNum], 1.3, 0, 1.6);

            /* Keeps track of sphere number */
            cNum ++;
        }
        else
        {
            /* No more sphere spawning */
            doneCreation = 1;
        }

        /* Do all colisions */
        dSpaceCollide (space, 0, &nearCallback);

        /* One step in simulation */
        dWorldQuickStep (world, STEP);

        /* remove all contact joints */
        dJointGroupEmpty (contactgroup);

        /* Adds a random brownian motion to spheres */
        for (i=0; i < (doneCreation? NUM : cNum); i++)
        {
            dReal *p = (dReal*)dBodyGetPosition(body[i]);
            if (p[2] < BOXH)
                dBodyAddForce(body[i],
                              (rand() % 200)/100.0f -1.,
                              (rand() % 200)/100.0f -1.,
                              (rand() % 200)/100.0f -1.);
        }

        /* Changes height of base water to rise a bit */
        dGeomSetPosition(g5, 10 + (BOXW/2.), 0., iter*BOXD/2000.0f);
    
        /* write out frame data: sphere positions */
        char filename[16];

        /* Create a numbered date file name */
        sprintf(filename, "data/%04d.dat", iter);
        FILE *arq = fopen(filename, "w");

        /* print into file the number of spheres */
        fprintf(arq, "%d", (doneCreation? NUM : cNum));

        /* Print each sphere coordinate */
        for (i=0; i < (doneCreation? NUM : cNum); i++)
        {
            bodypos = (dReal*)dBodyGetPosition(body[i]);
            fprintf(arq, ", <%f, %f, %f>", bodypos[0], bodypos[1], bodypos[2]);
        }
        fprintf(arq, "\n");
        fclose(arq);
    }

    #ifdef GRAPHICS
    /* Set color to water color */
    dsSetColor (0, 1, 1);

    /* Draw each water sphere */
    for (i=0; i < (doneCreation? NUM : cNum); i++)
        dsDrawSphere (dBodyGetPosition(body[i]),
                dBodyGetRotation(body[i]), 2*RADIUS);

    /* Draw boxes */
    #if DRAW_BOXES
    dReal *bodyr;

    bodypos = (dReal*)dGeomGetPosition (g5);
    bodyr = (dReal*)dGeomGetRotation (g5);
    dsDrawBox(bodypos, bodyr, s5);

    dsSetColor (.4, .4, .4);
    bodypos = dGeomGetPosition (g1);
    bodyr = dGeomGetRotation (g1);
    dsDrawBox(bodypos, bodyr, s1);

    bodypos = dGeomGetPosition (g2);
    bodyr = dGeomGetRotation (g2);
    dsDrawBox(bodypos, bodyr, s2);

    bodypos = dGeomGetPosition (g3);
    bodyr = dGeomGetRotation (g3);
    dsDrawBox(bodypos, bodyr, s3);

    bodypos = dGeomGetPosition (g4);
    bodyr = dGeomGetRotation (g4);
    dsDrawBox(bodypos, bodyr, s4);
    #endif

    #else
    printf("%04d\n", iter);
    #endif
}

int main (int argc, char **argv)
{
    srand(time(NULL));

    #ifdef GRAPHICS
    /* setup pointers to drawstuff callback functions */
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = 0;
    fn.stop = 0;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    #endif

    /* create world */
    dInitODE2(0);
    world = dWorldCreate();
    space = dHashSpaceCreate (0);
    contactgroup = dJointGroupCreate (1000000);
    dWorldSetGravity (world,0,0,-0.5);
    dCreatePlane (space,0,0,1,0);

    /* create geom */
    g1 = dCreateBox(space,  BOXD, BOXW, BOXH);
    g2 = dCreateBox(space,  BOXD, BOXW, BOXH);
    g3 = dCreateBox(space,  BOXW, BOXD, BOXH);
    g4 = dCreateBox(space,  BOXW, BOXD, BOXH);
    g5 = dCreateBox(space,  BOXW, BOXW, BOXD);

    /* set geometry */
    dGeomSetBody (g1, 0);
    dGeomSetBody (g2, 0);
    dGeomSetBody (g3, 0);
    dGeomSetBody (g4, 0);
    dGeomSetBody (g5, 0);

    /* set position */
    dGeomSetPosition(g1, 10 + BOXW, 0, BOXH/2.);
    dGeomSetPosition(g2, 10, 0, BOXH/2.);
    dGeomSetPosition(g3, 10 + (BOXW/2.), BOXW/2, BOXH/2.);
    dGeomSetPosition(g4, 10 + (BOXW/2.), -BOXW/2., BOXH/2.);
    dGeomSetPosition(g5, 10 + (BOXW/2.), 0., 0.);

    cNum = 0;
    doneCreation = 0;

    #ifdef GRAPHICS

    /* run simulation loop with graphics using drawstuff */
    dsSimulationLoop (argc,argv, 800, 600, &fn);

    #else
    start();

    /* Loop until there is 1000 ticks after sphere(NUM) creations */
    int i;
    for (i = 0; i < NUM+1000; i++)
        simLoop(0);

    #endif

    /* Frees memory and close ODE */
    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (space);
    dWorldDestroy (world);
    dCloseODE();

    return 0;
}

