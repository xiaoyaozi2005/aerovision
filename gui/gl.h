#ifndef LGLWIDGET_H
#define LGLWIDGET_H


#include <kqt.h>
#include <kqtgl.h>

#include "../modules/controller.h"
#define LIGHT_NUM 2

class LGLWidget : public KQt::GLWidget
{
Q_OBJECT

 KQt::GL::Light light[LIGHT_NUM];
 KQt::GL::SerialMechanismDrawer drawerFK;
 KQt::GL::SerialMechanismDrawer drawerIK;
 
 public:
 LGLWidget();
 ~LGLWidget();

 void create() override;
 void draw() override;


};


#endif
