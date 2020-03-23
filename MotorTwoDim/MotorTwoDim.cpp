/*======================================================================================================================

    Copyright 2011, 2012, 2013, 2014, 2015 Institut fuer Neuroinformatik, Ruhr-Universitaet Bochum, Germany

    This file is part of cedar.

    cedar is free software: you can redistribute it and/or modify it under
    the terms of the GNU Lesser General Public License as published by the
    Free Software Foundation, either version 3 of the License, or (at your
    option) any later version.

    cedar is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
    License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with cedar. If not, see <http://www.gnu.org/licenses/>.

========================================================================================================================

    Institute:   Ruhr-Universitaet Bochum
                 Institut fuer Neuroinformatik

    File:        EarSubscriber.h

    Maintainer:  Tutorial Writer Person
    Email:       cedar@ini.rub.de
    Date:        2011 12 09

    Description:

    Credits:

======================================================================================================================*/

// CEDAR INCLUDES
#include "MotorTwoDim.h"
#include <cedar/processing/ExternalData.h> // getInputSlot() returns ExternalData
#include <cedar/auxiliaries/MatData.h> // this is the class MatData, used internally in this step
#include "cedar/auxiliaries/math/functions.h"
#include <cmath>
#include <iostream>

using namespace std;





// SYSTEM INCLUDES

//----------------------------------------------------------------------------------------------------------------------
// constructors and destructor
//----------------------------------------------------------------------------------------------------------------------
MotorTwoDim::MotorTwoDim()
:
cedar::proc::Step(true),
mInputX(new cedar::aux::MatData(cv::Mat::zeros(10, 10, CV_32F))),
mTopic(new cedar::aux::StringParameter(this, "Yarp Destination", "")),
mTolerance(new cedar::aux::IntParameter(this, "Tolerance New Motion",1)),
mFixedZ(new cedar::aux::DoubleParameter(this, "fixed Z",0.2)),
mReconnect(new cedar::aux::DoubleParameter(this,"reconnect",0.0)),
mLowerX(new cedar::aux::DoubleParameter(this,"lower x",-1.0)),
mUpperX(new cedar::aux::DoubleParameter(this,"upper x",1.0)),
mLowerY(new cedar::aux::DoubleParameter(this,"lower y",-1.0)),
mUpperY(new cedar::aux::DoubleParameter(this,"upper y",1.0))
{
this->declareInput("F", true);

valPeak = 0;
upper_x = 1;
lower_x = -1;
upper_y = 1;
lower_y = -1;

field_pos = 0;
minX = sizeX;
minY = sizeY;
maxX = 0;
maxY = 0;
toler = 1;
this->connect(this->mTopic.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
this->connect(this->mReconnect.get(), SIGNAL(valueChanged()), this, SLOT(reConnectYarp()));
this->connect(this->mFixedZ.get(), SIGNAL(valueChanged()), this, SLOT(fixZ()));
this->connect(this->mTolerance.get(), SIGNAL(valueChanged()), this, SLOT(reCompute()));
this->connect(this->mLowerX.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
this->connect(this->mUpperX.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
this->connect(this->mLowerY.get(), SIGNAL(valueChanged()), this, SLOT(reName()));
this->connect(this->mUpperY.get(), SIGNAL(valueChanged()), this, SLOT(reName()));

}
//----------------------------------------------------------------------------------------------------------------------
// methods
//----------------------------------------------------------------------------------------------------------------------
//
void MotorTwoDim::compute(const cedar::proc::Arguments&)
{

  cv::Mat& field = mInputX->getData();
  cedar::aux::ConstDataPtr opX = this->getInputSlot("F")->getData();
  field = opX->getData<cv::Mat>();
  cv::Size s = field.size();
  sizeX = s.width;
  sizeY = s.height;

  maxX = 0;
  maxY = 0;
  minX = sizeX;
  minY = sizeY;

  for(int i = 0;i < sizeY;i++)
  {
    for(int j = 0;j < sizeX;j++)
    {
      valPeak = field.at<float>(i,j);
      if(valPeak > 0.8)
      {
        if(j < minX)
        {
          minX = j;
        }
        if(i < minY)
        {
          minY = i;
        }
        if(j > maxX)
        {
          maxX = j;
        }
        if(i > maxY)
        {
          maxY = i;
        }
      }
    }
  }
  posX = (static_cast<double>(minX) + static_cast<double>(maxX))/2;
  posY = (static_cast<double>(minY) + static_cast<double>(maxY))/2;

  float tolX = std::abs(posX - old_posX);
  float tolY = std::abs(posY - old_posY);

  if(tolX >= toler || tolY >= toler)
  {
     float cartX = setPositionX(posX);
     float cartY = setPositionY(posY);
     Bottle&out = outPort.prepare();
     out.clear();
     out.addFloat32(cartX);
     out.addFloat32(cartY);
     out.addFloat32(cartZ);
     outPort.write(true);
  }

  old_posX = posX;
  old_posY = posY;


}

//simple function to transform coordinates from field to angle

double MotorTwoDim::setPositionX(double position)
{
  double p = position / sizeX;
  double a = (upper_x - lower_x);
  double res = a*p + lower_x;

  return res;
}

double MotorTwoDim::setPositionY(double position)
{
  double p = position / sizeY;
  double a = (upper_y - lower_y);
  double res = a*p + lower_y;

  return res;
}

//specificc to this size - angle to field coordinates transform

double MotorTwoDim::getPositionX(double position)
{
   double a = sizeX / (upper_x - lower_x);
   double res = position * a + 50;

   return res;
}

double MotorTwoDim::getPositionY(double position)
{
   double a = sizeY / (upper_y - lower_y);
   double res = position * a + 50;

   return res;
}

void MotorTwoDim::reCompute()
{
   toler = this->mTolerance->getValue();
   minX = sizeX;
   minY = sizeY;

}

void MotorTwoDim::reName()
{
   inputPort = this->mTopic->getValue();
   lower_x = this->mLowerX->getValue();
   upper_x = this->mUpperX->getValue();
   lower_y = this->mLowerY->getValue();
   upper_y = this->mUpperY->getValue();
}

void MotorTwoDim::reConnectYarp()
{
  inPort.open(inputPort);
  outPort.open("/cedar/motor");
  yarp.connect(outPort.getName(),inPort.getName());
}

void MotorTwoDim::fixZ()
{
   cartZ = this->mFixedZ->getValue();
}

void MotorTwoDim::reset()
{

}
