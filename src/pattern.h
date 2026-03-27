/**
 *   Patterns of the StrokeEngine
 *   A library to create a variety of stroking motions with a stepper or servo motor on an ESP32.
 *   https://github.com/theelims/StrokeEngine 
 *
 * Copyright (C) 2021 theelims <elims@gmx.net>
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#pragma once

#include <Arduino.h>
#include <StrokeEngine.h>
#include <math.h>
#include "PatternMath.h"

#define DEBUG_PATTERN                 // Print some debug informations over Serial

#ifndef STRING_LEN
  #define STRING_LEN           64     // Bytes used to initialize char array. No path, topic, name, etc. should exceed this value
#endif

/**************************************************************************/
/*!
  @brief  struct to return all parameters FastAccelStepper needs to calculate
  the trapezoidal profile.
*/
/**************************************************************************/
typedef struct {
    int stroke;         //!< Absolute and properly constrainted target position of a move in steps 
    int speed;          //!< Speed of a move in Steps/second 
    int acceleration;   //!< Acceleration to get to speed or halt 
    bool skip;          //!< no valid stroke, skip this set an query for the next --> allows pauses between strokes
} motionParameter;


/**************************************************************************/
/*!
  @class Pattern 
  @brief  Base class to derive your pattern from. Offers a unified set of
          functions to store all relevant paramteres. These function can be
          overridenid necessary. Pattern should be self-containted and not 
          rely on any stepper/servo related properties. Internal book keeping
          is done in steps. The translation from real word units to steps is
          provided by the StrokeEngine. Also the sanity check whether motion
          parameters are physically possible are done by the StrokeEngine. 
          Imposible motion commands are clipped, cropped or adjusted while 
          still having a smooth appearance.  
*/
/**************************************************************************/
class Pattern {

    public:
        //! Constructor
        /*!
          @param str String containing the name of a pattern 
        */
        Pattern(const char *str) { strcpy(_name, str); }

        //! Set the time a normal stroke should take to complete
        /*! 
          @param speed time of a full stroke in [sec] 
        */
        virtual void setTimeOfStroke(float speed) { _timeOfStroke = speed; }

        //! Set the maximum stroke a pattern may have
        /*! 
          @param stroke stroke distance in Steps 
        */
        virtual void setStroke(int stroke) { _stroke = stroke; }

        //! Set the maximum depth a pattern may have
        /*! 
          @param stroke stroke distance in Steps 
        */
        virtual void setDepth(int depth) { _depth = depth; }

        //! Sensation is an additional parameter a pattern can take to alter its behaviour
        /*! 
          @param sensation Arbitrary value from -100 to 100, with 0 beeing neutral 
        */
        virtual void setSensation(float sensation) { _sensation = sensation; } 

        //! Retrives the name of a pattern
        /*! 
          @return c_string containing the name of a pattern 
        */
        char *getName() { return _name; }

        //! Calculate the position of the next stroke based on the various parameters
        /*! 
          @param index index of a stroke. Increments with every new stroke. 
          @return Set of motion parameteres like speed, acceleration & position
        */
        virtual motionParameter nextTarget(unsigned int index) {
            _index = index;
            return _nextMove;
        } 

        //! Communicates the maximum possible speed and acceleration limits of the machine to a pattern.
        /*! 
          @param maxSpeed maximum speed which is possible. Higher speeds get truncated inside StrokeEngine anyway.
          @param maxAcceleration maximum possible acceleration. Get also truncated, if impossible.
          @param stepsPerMM 
        */
        virtual void setSpeedLimit(unsigned int maxSpeed, unsigned int maxAcceleration, unsigned int stepsPerMM) { _maxSpeed = maxSpeed; _maxAcceleration = maxAcceleration; _stepsPerMM = stepsPerMM; } 

    protected:
        int _stroke;
        int _depth;
        float _timeOfStroke;
        float _sensation = 0.0;
        int _index = -1;
        char _name[STRING_LEN]; 
        motionParameter _nextMove = {0, 0, 0, false};
        int _startDelayMillis = 0;
        int _delayInMillis = 0;
        unsigned int _maxSpeed = 0;
        unsigned int _maxAcceleration = 0;
        unsigned int _stepsPerMM = 0;

        /*!
          @brief Start a delay timer which can be polled by calling _isStillDelayed(). 
          Uses internally the millis()-function.
        */
        void _startDelay() {
            _startDelayMillis = millis();
        } 

        /*! 
          @brief Update a delay timer which can be polled by calling _isStillDelayed(). 
          Uses internally the millis()-function.
          @param delayInMillis delay in milliseconds 
        */
        void _updateDelay(int delayInMillis) {
            _delayInMillis = delayInMillis;
        } 

        /*! 
          @brief Poll the state of a internal timer to create pauses between strokes. 
          Uses internally the millis()-function.
          @return True, if the timer is running, false if it is expired.
        */
        bool _isStillDelayed() {
            return (millis() > (_startDelayMillis + _delayInMillis)) ? false : true; 
        }

};

/**************************************************************************/
/*!
  @brief  Simple Stroke Pattern. It creates a trapezoidal stroke profile
  with 1/3 acceleration, 1/3 coasting, 1/3 deceleration. Sensation has 
  no effect.
*/
/**************************************************************************/
class SimpleStroke : public Pattern {
    public:
        SimpleStroke(const char *str) : Pattern(str) {}

        void setTimeOfStroke(float speed = 0) { 
             // In & Out have same time, so we need to divide by 2
            _timeOfStroke = 0.5 * speed; 
        }   

        motionParameter nextTarget(unsigned int index) {
            // maximum speed of the trapezoidal motion 
            _nextMove.speed = int(1.5 * _stroke/_timeOfStroke);

            // acceleration to meet the profile
            _nextMove.acceleration = int(3.0 * _nextMove.speed/_timeOfStroke);

            // odd stroke is moving out    
            if (index % 2) {
                _nextMove.stroke = _depth - _stroke;
            
            // even stroke is moving in
            } else {
                _nextMove.stroke = _depth;
            }

            _index = index;
            return _nextMove;
        }
};

/**************************************************************************/
/*!
  @brief  Simple pattern where the sensation value can change the speed 
  ratio between in and out. Sensation > 0 make the in move faster (up to 5x)
  giving a hard pounding sensation. Values < 0 make the out move going 
  faster. This gives a more pleasing sensation. The time for the overall 
  stroke remains the same. 
*/
/**************************************************************************/
class TeasingPounding : public Pattern {
    public:
        TeasingPounding(const char *str) : Pattern(str) {}
        void setSensation(float sensation) { 
            _sensation = sensation;
            _updateStrokeTiming();
        }
        void setTimeOfStroke(float speed = 0) {
            _timeOfStroke = speed;
            _updateStrokeTiming();
        }
        motionParameter nextTarget(unsigned int index) {
            // odd stroke is moving out
            if (index % 2) {
                // maximum speed of the trapezoidal motion
                _nextMove.speed = int(1.5 * _stroke/_timeOfOutStroke);

                // acceleration to meet the profile                  
                _nextMove.acceleration = int(3.0 * float(_nextMove.speed)/_timeOfOutStroke);    
                _nextMove.stroke = _depth - _stroke;
            // even stroke is moving in
            } else {
                // maximum speed of the trapezoidal motion
                _nextMove.speed = int(1.5 * _stroke/_timeOfInStroke); 
     
                // acceleration to meet the profile            
                _nextMove.acceleration = int(3.0 * float(_nextMove.speed)/_timeOfInStroke);    
                _nextMove.stroke = _depth;
            }
            _index = index;
            return _nextMove;
        }
    protected:
        float _timeOfFastStroke = 1.0;
        float _timeOfInStroke = 1.0;
        float _timeOfOutStroke = 1.0;
        void _updateStrokeTiming() {
            // calculate the time it takes to complete the faster stroke
            // Division by 2 because reference is a half stroke
            _timeOfFastStroke = (0.5 * _timeOfStroke) / fscale(0.0, 100.0, 1.0, 5.0, abs(_sensation), 0.0);
            // positive sensation, in is faster
            if (_sensation > 0.0) {
                _timeOfInStroke = _timeOfFastStroke;
                _timeOfOutStroke = _timeOfStroke - _timeOfFastStroke;
            // negative sensation, out is faster
            } else {
                _timeOfOutStroke = _timeOfFastStroke;
                _timeOfInStroke = _timeOfStroke - _timeOfFastStroke;
            }
#ifdef DEBUG_PATTERN
            Serial.println("TimeOfInStroke: " + String(_timeOfInStroke));
            Serial.println("TimeOfOutStroke: " + String(_timeOfOutStroke));
#endif
        }
};


/**************************************************************************/
/*!
  @brief  Robot Stroke Pattern. Sensation controls the acceleration of the
  stroke. Positive value increase acceleration until it is a constant speed
  motion (feels robotic). Neutral is equal to simple stroke (1/3, 1/3, 1/3).
  Negative reduces acceleration into a triangle profile.
*/
/**************************************************************************/ 
class RoboStroke : public Pattern {
    public:
        RoboStroke(const char *str) : Pattern(str) {}

        void setTimeOfStroke(float speed = 0) { 
             // In & Out have same time, so we need to divide by 2
            _timeOfStroke = 0.5 * speed; 
        }

        void setSensation(float sensation = 0) { 
            _sensation = sensation;
            // scale sensation into the range [0.05, 0.5] where 0 = 1/3
            if (sensation >= 0 ) {
              _x = fscale(0.0, 100.0, 1.0/3.0, 0.5, sensation, 0.0);
            } else {
              _x = fscale(0.0, 100.0, 1.0/3.0, 0.05, -sensation, 0.0);
            }
#ifdef DEBUG_PATTERN
            Serial.println("Sensation:" + String(sensation,0) + " --> " + String(_x,6));
#endif
        }

        motionParameter nextTarget(unsigned int index) {
            // maximum speed of the trapezoidal motion
            float speed = float(_stroke) / ((1 - _x) * _timeOfStroke);
            _nextMove.speed = int(speed); 

            // acceleration to meet the profile
            _nextMove.acceleration = int(speed / (_x * _timeOfStroke));

            // odd stroke is moving out    
            if (index % 2) {
                _nextMove.stroke = _depth - _stroke;
            
            // even stroke is moving in
            } else {
                _nextMove.stroke = _depth;
            }

            _index = index;
            return _nextMove;
        }
    protected:
        float _x = 1.0/3.0;
};

/**************************************************************************/
/*!
  @brief  Like Teasing or Pounding, but every second stroke is only half the
  depth. The sensation value can change the speed ratio between in and out. 
  Sensation > 0 make the in move faster (up to 5x) giving a hard pounding 
  sensation. Values < 0 make the out move going faster. This gives a more 
  pleasing sensation. The time for the overall stroke remains the same for
  all strokes, even half ones. 
*/
/**************************************************************************/
class HalfnHalf : public Pattern {
    public:
        HalfnHalf(const char *str) : Pattern(str) {}
        void setSensation(float sensation) { 
            _sensation = sensation;
            _updateStrokeTiming();
        }
        void setTimeOfStroke(float speed = 0) {
            _timeOfStroke = speed;
            _updateStrokeTiming();
        }
        motionParameter nextTarget(unsigned int index) {
            // check if this is the very first 
            if (index == 0) {
              //pattern started for the very fist time, so we start gentle with a half move
              _half = true;
            }

            // set-up the stroke length
            int stroke = _stroke;
            if (_half == true) {
                // half the stroke length
                stroke = _stroke / 2;
            } 

            // odd stroke is moving out
            if (index % 2) {
                // maximum speed of the trapezoidal motion
                _nextMove.speed = int(1.5 * stroke/_timeOfOutStroke);  

                // acceleration to meet the profile                  
                _nextMove.acceleration = int(3.0 * float(_nextMove.speed)/_timeOfOutStroke);    
                _nextMove.stroke = _depth - _stroke;
                // every second move is half
                _half = !_half;
            // even stroke is moving in
            } else {
                // maximum speed of the trapezoidal motion
                _nextMove.speed = int(1.5 * stroke/_timeOfInStroke);  
     
                // acceleration to meet the profile            
                _nextMove.acceleration = int(3.0 * float(_nextMove.speed)/_timeOfInStroke);    
                _nextMove.stroke = (_depth - _stroke) + stroke;  
            }
            _index = index;
            return _nextMove;
        }
    protected:
        float _timeOfFastStroke = 1.0;
        float _timeOfInStroke = 1.0;
        float _timeOfOutStroke = 1.0;
        bool _half = true;
        void _updateStrokeTiming() {
            // calculate the time it takes to complete the faster stroke
            // Division by 2 because reference is a half stroke
            _timeOfFastStroke = (0.5 * _timeOfStroke) / fscale(0.0, 100.0, 1.0, 5.0, abs(_sensation), 0.0);
            // positive sensation, in is faster
            if (_sensation > 0.0) {
                _timeOfInStroke = _timeOfFastStroke;
                _timeOfOutStroke = _timeOfStroke - _timeOfFastStroke;
            // negative sensation, out is faster
            } else {
                _timeOfOutStroke = _timeOfFastStroke;
                _timeOfInStroke = _timeOfStroke - _timeOfFastStroke;
            }
#ifdef DEBUG_PATTERN
            Serial.println("TimeOfInStroke: " + String(_timeOfInStroke));
            Serial.println("TimeOfOutStroke: " + String(_timeOfOutStroke));
#endif
        }
};

/**************************************************************************/
/*!
  @brief  The insertion depth ramps up gradually with each stroke until it
  reaches its maximum. It then resets and restars. Sensations controls how 
  many strokes there are in a ramp.
*/
/**************************************************************************/
class Deeper : public Pattern {
    public:
        Deeper(const char *str) : Pattern(str) {}

        void setTimeOfStroke(float speed = 0) { 
             // In & Out have same time, so we need to divide by 2
            _timeOfStroke = 0.5 * speed; 
        }   

        void setSensation(float sensation) { 
            _sensation = sensation;

            // maps sensation to useful values [2,22] with 12 beeing neutral
            if (sensation < 0) {
                _countStrokesForRamp = map(sensation, -100, 0, 2, 11);
            } else {
                _countStrokesForRamp = map(sensation, 0, 100, 11, 32);
            }
#ifdef DEBUG_PATTERN
            Serial.println("_countStrokesForRamp: " + String(_countStrokesForRamp));
#endif
        }

        motionParameter nextTarget(unsigned int index) {
            // How many steps is each stroke advancing         
            int slope = _stroke / (_countStrokesForRamp);

            // The pattern recycles so we use modulo to get a cycling index.
            // Factor 2 because index increments with each full stroke twice
            // add 1 because modulo = 0 is index = 1
            int cycleIndex = (index / 2) % _countStrokesForRamp + 1;

            // This might be not smooth, as the insertion depth may jump when 
            // sensation is adjusted.

            // Amplitude is slope * cycleIndex
            int amplitude = slope * cycleIndex;
#ifdef DEBUG_PATTERN
            Serial.println("amplitude: " + String(amplitude)
                         + " cycleIndex: " + String(cycleIndex));
#endif

            // maximum speed of the trapezoidal motion 
            _nextMove.speed = int(1.5 * amplitude/_timeOfStroke); 

            // acceleration to meet the profile
            _nextMove.acceleration = int(3.0 * _nextMove.speed/_timeOfStroke);

            // odd stroke is moving out    
            if (index % 2) {
                _nextMove.stroke = _depth - _stroke;
            
            // even stroke is moving in
            } else {
                _nextMove.stroke = (_depth - _stroke) + amplitude;
            }

            _index = index;
            return _nextMove;
        }
    
    protected:
        int _countStrokesForRamp = 2;

};

/**************************************************************************/
/*!
  @brief  Pauses between a series of strokes. 
  The number of strokes ramps from 1 stroke to 5 strokes and back. Sensation 
  changes the length of the pauses between stroke series.
*/
/**************************************************************************/
class StopNGo : public Pattern {
    public:
        StopNGo(const char *str) : Pattern(str) {}

        void setTimeOfStroke(float speed = 0) { 
             // In & Out have same time, so we need to divide by 2
            _timeOfStroke = 0.5 * speed; 
        }   

        void setSensation(float sensation) { 
            _sensation = sensation;

            // maps sensation to a delay from 100ms to 10 sec
            _updateDelay(map(sensation, -100, 100, 100, 10000));
        }

        motionParameter nextTarget(unsigned int index) {
            // maximum speed of the trapezoidal motion 
            _nextMove.speed = int(1.5 * _stroke/_timeOfStroke); 

            // acceleration to meet the profile
            _nextMove.acceleration = int(3.0 * _nextMove.speed/_timeOfStroke);

            // adds a delay between each stroke
            if (_isStillDelayed() == false) {

                // odd stroke is moving out    
                if (index % 2) {
                    _nextMove.stroke = _depth - _stroke;

                    if (_strokeIndex >= _strokeSeriesIndex) {
                        // Reset stroke index to 1
                        _strokeIndex = 0;

                        // change count direction once we reached the maximum number of strokes
                        if (_strokeSeriesIndex >= _numberOfStrokes) {
                            _countStrokesUp = false;
                        }

                        // change count direction once we reached one stroke counting down
                        if (_strokeSeriesIndex <= 1) {
                            _countStrokesUp = true;
                        }

                        // increment or decrement strokes counter
                        if (_countStrokesUp == true) {
                            _strokeSeriesIndex++;
                        } else {
                            _strokeSeriesIndex--;
                        }

                        // start delay after having moved out
                        _startDelay();
                    }
                    

                // even stroke is moving in
                } else {
                    _nextMove.stroke = _depth;
                    // Increment stroke index by one
                    _strokeIndex++;
                }
                _nextMove.skip = false;
            } else {
                _nextMove.skip = true;
            }

            _index = index;
            
            return _nextMove;
        }

    protected:
        int _numberOfStrokes = 5;
        int _strokeSeriesIndex = 1;
        int _strokeIndex = 0;
        bool _countStrokesUp = true;

};

/**************************************************************************/
/*!
  @brief  Sensation reduces the effective stroke length while keeping the
  stroke speed constant to the full stroke. This creates interesting 
  vibrational pattern at higher sensation values. With positive sensation the
  strokes will wander towards the front, with negative values towards the back.
*/
/**************************************************************************/
class Insist : public Pattern {
    public:
        Insist(const char *str) : Pattern(str) {}   

        void setSensation(float sensation) { 
            _sensation = sensation;

            // make invert sensation and make into a fraction of the stroke distance
            _strokeFraction = (100 - abs(sensation))/100.0f;

            _strokeInFront = (sensation > 0) ? true : false;

            _updateStrokeTiming();
        }

        void setTimeOfStroke(float speed = 0) { 
             // In & Out have same time, so we need to divide by 2
            _timeOfStroke = 0.5 * speed;
            _updateStrokeTiming();
        }   

        void setStroke(int stroke) {
            _stroke = stroke;
            _updateStrokeTiming();
        }

        motionParameter nextTarget(unsigned int index) {

            // acceleration & speed to meet the profile
            _nextMove.acceleration = _acceleration;
            _nextMove.speed = _speed;

            if (_strokeInFront) {
                // odd stroke is moving out
                if (index % 2) {
                    _nextMove.stroke = _depth - _realStroke;

                // even stroke is moving in
                } else {
                    _nextMove.stroke = _depth;  
                }

            } else {
                // odd stroke is moving out    
                if (index % 2) {
                    _nextMove.stroke = _depth - _stroke;
                    
                // even stroke is moving in
                } else {
                    _nextMove.stroke = (_depth - _stroke) + _realStroke;                
                }
            }

            _index = index;
            
            return _nextMove;
        }

    protected:
        int _speed = 0;
        int _acceleration = 0;
        int _realStroke = 0;
        float _strokeFraction = 1.0;
        bool _strokeInFront = false;
        void _updateStrokeTiming() {
            // maximum speed of the longest trapezoidal motion (full stroke)
            _speed = int(1.5 * _stroke/_timeOfStroke);

            // Acceleration to hold 1/3 profile with fractional strokes
            _acceleration = int(3.0 * _nextMove.speed/(_timeOfStroke * _strokeFraction));

            // Calculate fractional stroke length
            _realStroke = int((float)_stroke * _strokeFraction);
        }

};

/**************************************************************************/
/*!
  @brief  Fast vibration pattern. Creates rapid short-stroke oscillations
  centered at the depth position. Uses maximum speed and acceleration for
  the fastest possible back-and-forth motion.

  Sensation controls vibration amplitude:
    0     = full stroke length (same as Simple Stroke)
    ±100  = minimum ~2mm vibration
  Positive sensation centers vibration at the front (depth end),
  negative sensation centers it at the back (retracted end).

  Speed (FPM) still controls the overall cycle rate, but because the
  stroke is short the motor will hit max speed/accel and go as fast as
  it physically can.
*/
/**************************************************************************/
class Vibrate : public Pattern {
    public:
        Vibrate(const char *str) : Pattern(str) {}

        void setSensation(float sensation) {
            _sensation = sensation;
            _updateVibration();
        }

        void setTimeOfStroke(float speed = 0) {
            _timeOfStroke = 0.5 * speed;
        }

        void setStroke(int stroke) {
            _stroke = stroke;
            _updateVibration();
        }

        void setDepth(int depth) {
            _depth = depth;
            _updateVibration();
        }

        motionParameter nextTarget(unsigned int index) {
            // Use max speed and acceleration for fastest possible vibration
            _nextMove.speed = _maxSpeed > 0 ? _maxSpeed : int(1.5 * _vibrationAmplitude / _timeOfStroke);
            _nextMove.acceleration = _maxAcceleration > 0 ? _maxAcceleration : int(3.0 * _nextMove.speed / _timeOfStroke);

            // odd stroke moves to back position
            if (index % 2) {
                _nextMove.stroke = _vibrationCenter - _vibrationAmplitude / 2;
            // even stroke moves to front position
            } else {
                _nextMove.stroke = _vibrationCenter + _vibrationAmplitude / 2;
            }

            _index = index;
            return _nextMove;
        }

    protected:
        int _vibrationAmplitude = 10;
        int _vibrationCenter = 0;

        void _updateVibration() {
            // Map sensation to amplitude fraction: 0 = full stroke, ±100 = ~2mm
            float absSensation = abs(_sensation);

            // Scale: at sensation=0, fraction=1.0 (full stroke)
            //        at sensation=±100, fraction approaches minimum (2mm worth of steps)
            float fraction = 1.0 - (absSensation / 100.0) * 0.95;  // range [0.05, 1.0]
            _vibrationAmplitude = max(int((float)_stroke * fraction), (int)(2.0 * _stepsPerMM));

            // Positive sensation: vibrate near front (depth end)
            // Negative sensation: vibrate near back (retracted end)
            if (_sensation >= 0) {
                // Center at depth, pull back by half amplitude
                _vibrationCenter = _depth - _vibrationAmplitude / 2;
            } else {
                // Center at back of stroke, push forward by half amplitude
                _vibrationCenter = (_depth - _stroke) + _vibrationAmplitude / 2;
            }

            // Constrain center so vibration stays within [depth-stroke, depth]
            int minPos = _depth - _stroke + _vibrationAmplitude / 2;
            int maxPos = _depth - _vibrationAmplitude / 2;
            if (minPos > maxPos) {
                // stroke is smaller than amplitude, just center it
                _vibrationCenter = _depth - _stroke / 2;
            } else {
                if (_vibrationCenter < minPos) _vibrationCenter = minPos;
                if (_vibrationCenter > maxPos) _vibrationCenter = maxPos;
            }

#ifdef DEBUG_PATTERN
            Serial.println("Vibrate: amplitude=" + String(_vibrationAmplitude)
                         + " center=" + String(_vibrationCenter)
                         + " sensation=" + String(_sensation));
#endif
        }
};

/**************************************************************************/
/*!
  @brief  Jack Hammer pattern. Rapid, hard-hitting strokes that slam to
  full depth and retract quickly, like a jackhammer. The in-stroke is
  very fast while the out-stroke is slower, creating an aggressive
  pounding feel.

  Sensation controls the aggression ratio:
    -100  = equal speed in/out (smooth)
      0   = in-stroke 3x faster than out (default hammer)
    +100  = in-stroke 6x faster than out (maximum impact)
*/
/**************************************************************************/
class JackHammer : public Pattern {
    public:
        JackHammer(const char *str) : Pattern(str) {}

        void setSensation(float sensation) {
            _sensation = sensation;
            _updateTiming();
        }

        void setTimeOfStroke(float speed = 0) {
            _timeOfStroke = speed;
            _updateTiming();
        }

        motionParameter nextTarget(unsigned int index) {
            // odd stroke is retracting (out) — slower
            if (index % 2) {
                _nextMove.speed = int(1.5 * _stroke / _timeOfOutStroke);
                _nextMove.acceleration = int(3.0 * float(_nextMove.speed) / _timeOfOutStroke);
                _nextMove.stroke = _depth - _stroke;
            // even stroke is slamming in — fast
            } else {
                _nextMove.speed = int(1.5 * _stroke / _timeOfInStroke);
                _nextMove.acceleration = int(6.0 * float(_nextMove.speed) / _timeOfInStroke);
                _nextMove.stroke = _depth;
            }
            _nextMove.skip = false;
            _index = index;
            return _nextMove;
        }

    protected:
        float _timeOfInStroke = 0.1;
        float _timeOfOutStroke = 0.5;

        void _updateTiming() {
            // Map sensation to speed ratio [1, 6] where 0 → 3x
            float ratio = fscale(0.0, 100.0, 1.0, 6.0, abs(_sensation), 0.0);
            if (_sensation < 0) {
                // Negative: closer to equal speed (ratio → 1)
                ratio = fscale(0.0, 100.0, 3.0, 1.0, abs(_sensation), 0.0);
            } else {
                // Positive: more aggressive (ratio 3→6)
                ratio = fscale(0.0, 100.0, 3.0, 6.0, _sensation, 0.0);
            }
            // total time is _timeOfStroke; split between in and out
            _timeOfInStroke = _timeOfStroke / (1.0 + ratio);
            _timeOfOutStroke = _timeOfStroke - _timeOfInStroke;
#ifdef DEBUG_PATTERN
            Serial.println("JackHammer ratio=" + String(ratio)
                         + " in=" + String(_timeOfInStroke)
                         + " out=" + String(_timeOfOutStroke));
#endif
        }
};

/**************************************************************************/
/*!
  @brief  Stroke Nibbler pattern. Takes small, rapid "nibbling" bites at
  different positions along the stroke length. Each nibble is a short
  back-and-forth motion, and after a set of nibbles the position advances
  deeper (or retreats), creating a progressive teasing effect.

  Sensation controls nibble size and count:
    -100  = large nibbles (40% of stroke), 2 per position — aggressive
      0   = medium nibbles (20% of stroke), 4 per position — balanced
    +100  = tiny nibbles (5% of stroke), 8 per position — very teasing
*/
/**************************************************************************/
class StrokeNibbler : public Pattern {
    public:
        StrokeNibbler(const char *str) : Pattern(str) {}

        void setSensation(float sensation) {
            _sensation = sensation;
            _updateNibble();
        }

        void setTimeOfStroke(float speed = 0) {
            // Each nibble should be fast; base time is for one nibble cycle
            _timeOfStroke = speed;
            _nibbleTime = speed * 0.25;  // each nibble is 1/4 of full stroke time
        }

        void setStroke(int stroke) {
            _stroke = stroke;
            _updateNibble();
        }

        void setDepth(int depth) {
            _depth = depth;
            _updateNibble();
        }

        motionParameter nextTarget(unsigned int index) {
            // Which nibble position are we at? (cycles through _nibblesPerPos * 2 moves per position)
            int movesPerPosition = _nibblesPerPos * 2;
            int totalPositions = max(2, _stroke / max(_nibbleSize, 1));
            // Ping-pong: advance through positions then reverse
            int positionCycle = (index / movesPerPosition) % (totalPositions * 2);
            int positionIndex;
            if (positionCycle < totalPositions) {
                positionIndex = positionCycle;  // advancing
            } else {
                positionIndex = (totalPositions * 2) - positionCycle - 1;  // retreating
            }

            // Base position for this nibble cluster
            int basePos = (_depth - _stroke) + (positionIndex * _stroke / max(totalPositions - 1, 1));

            // Speed and acceleration for fast nibbles
            _nextMove.speed = int(1.5 * _nibbleSize / _nibbleTime);
            _nextMove.acceleration = int(3.0 * _nextMove.speed / _nibbleTime);

            // Alternate between base and base + nibbleSize
            if (index % 2) {
                _nextMove.stroke = max(basePos, _depth - _stroke);
            } else {
                _nextMove.stroke = min(basePos + _nibbleSize, _depth);
            }

            _nextMove.skip = false;
            _index = index;
            return _nextMove;
        }

    protected:
        int _nibbleSize = 10;
        int _nibblesPerPos = 4;
        float _nibbleTime = 0.25;

        void _updateNibble() {
            // Map sensation: -100→40% stroke, 0→20%, +100→5%
            float fraction;
            if (_sensation <= 0) {
                fraction = fscale(0.0, 100.0, 0.20, 0.40, abs(_sensation), 0.0);
            } else {
                fraction = fscale(0.0, 100.0, 0.20, 0.05, _sensation, 0.0);
            }
            _nibbleSize = max(int((float)_stroke * fraction), (int)(2.0 * _stepsPerMM));

            // Map sensation to nibbles per position: -100→2, 0→4, +100→8
            if (_sensation <= 0) {
                _nibblesPerPos = (int)fscale(0.0, 100.0, 4.0, 2.0, abs(_sensation), 0.0);
            } else {
                _nibblesPerPos = (int)fscale(0.0, 100.0, 4.0, 8.0, _sensation, 0.0);
            }
            _nibblesPerPos = max(_nibblesPerPos, 1);

#ifdef DEBUG_PATTERN
            Serial.println("StrokeNibbler: nibbleSize=" + String(_nibbleSize)
                         + " nibblesPerPos=" + String(_nibblesPerPos)
                         + " sensation=" + String(_sensation));
#endif
        }
};

/**************************************************************************/
/*
  Array holding all different patterns. Please include any custom pattern here.
*/
/**************************************************************************/
static Pattern *patternTable[] = { 
  new SimpleStroke("Simple Stroke"),
  new TeasingPounding("Teasing or Pounding"),
  new RoboStroke("Robo Stroke"),
  new HalfnHalf("Half'n'Half"),
  new Deeper("Deeper"),
  new StopNGo("Stop'n'Go"),
  new Insist("Insist"),
  new JackHammer("Jack Hammer"),
  new StrokeNibbler("Stroke Nibbler"),
  new Vibrate("Vibrate")
 };

static const unsigned int patternTableSize = sizeof(patternTable) / sizeof(patternTable[0]);