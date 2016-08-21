/*

 License

 Menge
 Copyright � and trademark � 2012-14 University of North Carolina at Chapel Hill.
 All rights reserved.

 Permission to use, copy, modify, and distribute this software and its documentation
 for educational, research, and non-profit purposes, without fee, and without a
 written agreement is hereby granted, provided that the above copyright notice,
 this paragraph, and the following four paragraphs appear in all copies.

 This software program and documentation are copyrighted by the University of North
 Carolina at Chapel Hill. The software program and documentation are supplied "as is,"
 without any accompanying services from the University of North Carolina at Chapel
 Hill or the authors. The University of North Carolina at Chapel Hill and the
 authors do not warrant that the operation of the program will be uninterrupted
 or error-free. The end-user understands that the program was developed for research
 purposes and is advised not to rely exclusively on the program for any reason.

 IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE AUTHORS
 BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
 AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
 DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY STATUTORY WARRANTY
 OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND
 THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS HAVE NO OBLIGATIONS
 TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

 Any questions or comments should be sent to the authors {menge,geom}@cs.unc.edu

 */

#include "MyViewer.h"
#include "GLScene.h"
#include "Logger.h"

namespace Menge {

namespace Vis {

/////////////////////////////////////////////////////////////////////////////
//                     Implementation of MyViewer
/////////////////////////////////////////////////////////////////////////////

MyViewer::MyViewer() :
		_scene(0x0), _stepSize(0.1f), _viewTime(0.f) {

}

/////////////////////////////////////////////////////////////////////////////

MyViewer::~MyViewer() {
	if (_scene) {
		delete _scene;
	}
}

/////////////////////////////////////////////////////////////////////////////

void MyViewer::setScene(SceneGraph::GLScene * scene) {
	if (_scene) {
		delete _scene;
	}
	_scene = scene;
}

///////////////////////////////////////////////////////////////////////////

void MyViewer::setFixedStep(float stepSize) {
	_stepSize = stepSize;
}

/////////////////////////////////////////////////////////////////////////////

void MyViewer::update(float simTime) {
        std::unique_lock<std::mutex> lock(sceneMutex);
        _viewTime = simTime;
       updateCV.notify_all(); 
}
void MyViewer::simLoop()
{
    float oldViewTime=0.0f;
    while(running)
    {
        std::unique_lock<std::mutex> lock(sceneMutex);
        
        while(_scene==0x0||_viewTime<=oldViewTime)
              updateCV.wait(lock);
        try {
                _scene->updateScene(_viewTime);
               oldViewTime=_viewTime;
        } catch (SceneGraph::SystemStopException ) {

        }
    }
}
void MyViewer::start() {
     running=true;
     pSimThread=new std::thread(&MyViewer::simLoop, this);
}
void MyViewer::stop() {
        sceneMutex.lock();
        running=false;
        sceneMutex.unlock();
        pSimThread->join(); 
	_scene->finish();
}
void MyViewer::pause() {
        sceneMutex.lock();
}

void MyViewer::resume() {
        sceneMutex.unlock();
        updateCV.notify_all();
}	

}	 // namespace Vis
}	// namespace Menge
