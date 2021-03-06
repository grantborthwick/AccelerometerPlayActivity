/*
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.example.android.accelerometerplay;

import java.util.ArrayList;
import java.util.List;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.BitmapFactory.Options;
import android.graphics.Color;
import android.graphics.Paint;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.PowerManager;
import android.os.PowerManager.WakeLock;
import android.util.DisplayMetrics;
import android.view.Display;
import android.view.Surface;
import android.view.View;
import android.view.WindowManager;
import android.widget.Toast;
import android.app.AlertDialog;

/**
 * This is an example of using the accelerometer to integrate the device's
 * acceleration to a position using the Verlet method. This is illustrated with
 * a very simple particle system comprised of a few iron balls freely moving on
 * an inclined wooden table. The inclination of the virtual table is controlled
 * by the device's accelerometer.
 * 
 * @see SensorManager
 * @see SensorEvent
 * @see Sensor
 */

public class AccelerometerPlayActivity extends Activity {

	final Context context = this;
	private SimulationView mSimulationView;
	private SensorManager mSensorManager;
	private PowerManager mPowerManager;
	private WindowManager mWindowManager;
	private Display mDisplay;
	private WakeLock mWakeLock;
    private boolean pause = false;

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		// Get an instance of the SensorManager
		mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);

		// Get an instance of the PowerManager
		mPowerManager = (PowerManager) getSystemService(POWER_SERVICE);

		// Get an instance of the WindowManager
		mWindowManager = (WindowManager) getSystemService(WINDOW_SERVICE);
		mDisplay = mWindowManager.getDefaultDisplay();

		// Create a bright wake lock
		mWakeLock = mPowerManager.newWakeLock(
				PowerManager.SCREEN_BRIGHT_WAKE_LOCK, getClass().getName());

		// instantiate our simulation view and set it as the activity's content
		Intent intent = getIntent();
		Bundle bundle = intent.getExtras();
		int CcX = bundle.getInt("CcX");
		int CcY = bundle.getInt("CcY");
		int level = bundle.getInt("level");//Fix show toast with level or show on screen.
		mSimulationView = new SimulationView(this,CcX,CcY);
		
		setContentView(mSimulationView);
	}

	@Override
	protected void onResume() {
		super.onResume();
		//pause = false;
		int mx = mSimulationView.mParticleSystem.mBalls[0].mBoxX;
		int my = mSimulationView.mParticleSystem.mBalls[0].mBoxY;
		mSimulationView.Boxes[mx][my].isTrap = false;
		pause = false;
		Toast toast = Toast.makeText(context, "You've fallen down a hole!", "You've fallen down a hole!".length());
		/*
		 * when the activity is resumed, we acquire a wake-lock so that the
		 * screen stays on, since the user will likely not be fiddling with the
		 * screen or buttons.
		 */
		mWakeLock.acquire();

		// Start the simulation
		mSimulationView.startSimulation();
	}

	@Override
	protected void onPause() {
		super.onPause();
		pause = true;
		/*
		 * When the activity is paused, we make sure to stop the simulation,
		 * release our sensor resources and wake locks
		 */

		// Stop the simulation
		mSimulationView.stopSimulation();

		// and release our wake-lock
		mWakeLock.release();
	}

	@Override
	public void onActivityResult(int requestCode, int resultCode, Intent data){
		super.onActivityResult(requestCode, resultCode, data);
		if(resultCode!=requestCode){
			finish();
		}
	}
	
	class SimulationView extends View implements SensorEventListener {
		// diameter of the balls in meters
		
		// friction of the virtual table and air
		private static final float sFriction = 0.1f;

		private Sensor mAccelerometer;
		private long mLastT;
		private float mLastDeltaT;
		private float mXDpi;
		private float mYDpi;
		private float mMetersToPixelsX;
		private float mMetersToPixelsY;
		private Bitmap mBitmap;
		private Bitmap mWood;
		private Paint line;
		private Paint line2;
		private Paint TrapPaint;
		private float mXOrigin;
		private float mYOrigin;
		private float mSensorX;
		private float mSensorY;
		private long mSensorTimeStamp;
		private long mCpuTimeStamp;
		private float mHorizontalBound;
		private float mVerticalBound;
		private float xc;
		private float yc;
		private float xs;
		private float ys;
		private final ParticleSystem mParticleSystem = new ParticleSystem();
		private Box[][] Boxes;
		private int CellCountX = 20;
		private int CellCountY = 30;
		private float sBallDiameter = 0.003f;
		private float sBallDiameter2 = sBallDiameter * sBallDiameter;
		private int TrapCount = (int)(CellCountX*CellCountY/7f);
		private float boxHeight;
		private float boxWidth;
		private DisplayMetrics metrics;

		/*
		 * Each of our particle holds its previous and current position, its
		 * acceleration. for added realism each particle has its own friction
		 * coefficient.
		 */
		class Particle {
			private float mPosX;
			private float mPosY;
			private float mAccelX;
			private float mAccelY;
			private float mLastPosX;
			private float mLastPosY;
			private float mOneMinusFriction;

			private int mBoxX;
			private int mBoxY;
			private int mLastBoxX;
			private int mLastBoxY;

			Particle() {
				// make each particle a bit different by randomizing its
				// coefficient of friction
				final float r = ((float) Math.random() - 0.5f) * 0.2f;
				mOneMinusFriction = 1.0f - sFriction + r;
				//Start particles out directly outside 0,0
				mPosX = -1;
				mPosY = 1;
			}

			public void computePhysics(float sx, float sy, float dT, float dTC) {
				// Force of gravity applied to our virtual object
				final float m = 1000.0f; // mass of our virtual object
				final float gx = -sx * m;
				final float gy = -sy * m;

				/*
				 * �F = mA <=> A = �F / m We could simplify the code by
				 * completely eliminating "m" (the mass) from all the equations,
				 * but it would hide the concepts from this sample code.
				 */
				final float invm = 1.0f / m;
				final float ax = gx * invm;
				final float ay = gy * invm;

				/*
				 * Time-corrected Verlet integration The position Verlet
				 * integrator is defined as x(t+�t) = x(t) + x(t) - x(t-�t) +
				 * a(t)�t�2 However, the above equation doesn't handle variable
				 * �t very well, a time-corrected version is needed: x(t+�t) =
				 * x(t) + (x(t) - x(t-�t)) * (�t/�t_prev) + a(t)�t�2 We also add
				 * a simple friction term (f) to the equation: x(t+�t) = x(t) +
				 * (1-f) * (x(t) - x(t-�t)) * (�t/�t_prev) + a(t)�t�2
				 */
				final float dTdT = dT * dT;
				final float x = mPosX + mOneMinusFriction * dTC	* (mPosX - mLastPosX) + mAccelX * dTdT;
				final float y = mPosY + mOneMinusFriction * dTC	* (mPosY - mLastPosY) + mAccelY * dTdT;
				mLastPosX = mPosX;
				mLastPosY = mPosY;
				mPosX = x;
				mPosY = y;
				mAccelX = ax;
				mAccelY = ay;
			}

			/*
			 * Resolving constraints and collisions with the Verlet integrator
			 * can be very simple, we simply need to move a colliding or
			 * constrained particle in such way that the constraint is
			 * satisfied.
			 */
			public void resolveCollisionWithBounds() {				
				float w = metrics.widthPixels;
				float h = metrics.heightPixels;
				float BoxW = w/CellCountX;
				float BoxH = h/CellCountY;
				float maxXPixels,minXPixels,maxYPixels,minYPixels;

				if(!Boxes[mBoxX][mBoxY].hasRight){maxXPixels = (mBoxX+1)*BoxW - w/2-1;}
				else{maxXPixels = (mBoxX+2)*BoxW - w/2-1;}
				if(!Boxes[mBoxX][mBoxY].hasLeft){minXPixels = (mBoxX)*BoxW - w/2+1;}
				else{minXPixels = (mBoxX-1)*BoxW - w/2+1;}
				if(!Boxes[mBoxX][mBoxY].hasUp){maxYPixels = (mBoxY)*BoxH - h/2+1;}
				else{maxYPixels = (mBoxY-1)*BoxH - h/2+1;}
				if(!Boxes[mBoxX][mBoxY].hasDown){minYPixels = (mBoxY+1)*BoxH - h/2+1;}
				else{minYPixels = (mBoxY+2)*BoxH - h/2+1;}

				float xmax = Math.min(maxXPixels/xs - sBallDiameter/2,mHorizontalBound);
				float xmin = Math.max(minXPixels/xs + sBallDiameter/2,-mHorizontalBound);
				float ymax = Math.min(-maxYPixels/ys - sBallDiameter/2,mVerticalBound);
				float ymin = Math.max(-minYPixels/ys + sBallDiameter/2,-mVerticalBound);

				float x = mPosX;
				float y = mPosY;
				mLastPosX=mPosX;
				mLastPosY=mPosY;
				//Might switch this back to main edges, or change to allow outside of bounds area.
				if (x > xmax) {	mPosX = xmax; } 
				else if (x < xmin) { mPosX = xmin; }
				if (y > ymax) {	mPosY = ymax; } 
				else if (y < ymin) { mPosY = ymin; }
				//Need to figure out which boxes must be passed through and then which borders need to be checked.
				int NewXBox = getBoxXFromPixel(xc + x*xs);
				int NewYBox = getBoxYFromPixel(yc - y*ys);
				/*if (NewXBox != mBoxX && NewYBox != mBoxY && mBoxX>=0 && mBoxY>=0 && NewXBox>=0&&NewYBox>=0){
					
				}
				else{*/
					mLastBoxX = mBoxX;
					mLastBoxY = mBoxY;
					mBoxX = getBoxXFromPixel(xc + mPosX*xs);
					mBoxY = getBoxYFromPixel(yc - mPosY*ys);
					if(Boxes[mBoxX][mBoxY].isTrap){
						if(!pause){
							Toast toast = Toast.makeText(context, "You've fallen down a hole!", "You've fallen down a hole!".length());
							toast.show();
							Intent intent = getIntent();
							Intent levelDown = new Intent(context, AccelerometerPlayActivity.class);
							Bundle parem = new Bundle();
							Bundle oldParem = intent.getExtras();
							parem.putInt("level", oldParem.getInt("level")+1);
							parem.putInt("CcY", CellCountY-1);
							parem.putInt("CcX", CellCountX-1);
							levelDown.putExtras(parem);//Fix parem
							((Activity)context).startActivityForResult(levelDown,1);
							pause=true;
						}
						
						/*mBoxX = 0;
						mBoxY = 0;
						mPosX = mHorizontalBound;
						mPosY = mVerticalBound;
						mLastBoxX = 0;
						mLastBoxY = 0;
						mLastPosX = mHorizontalBound-1;
						mLastPosY = mVerticalBound-1;
						mLastT = 0;
						mLastDeltaT = 0;
						GenerateMaze2(CellCountX,CellCountY, TrapCount, Boxes);*/
					}
					if(Boxes[mBoxX][mBoxY].isGoal){
						if(!pause){
							pause = true;
							Toast toast = Toast.makeText(context, "You've escaped one level up!", "You've escaped one level up!".length());
							toast.show();
							Intent intent = getIntent();
							setResult(1, intent);
							finish();
						}
					}
				//}
			}
		}

		/*
		 * A particle system is just a collection of particles
		 */
		class ParticleSystem {
			static final int NUM_PARTICLES = 1;
			public Particle mBalls[] = new Particle[NUM_PARTICLES];

			ParticleSystem() {
				/*
				 * Initially our particles have no speed or acceleration
				 */
				for (int i = 0; i < mBalls.length; i++) {
					mBalls[i] = new Particle();
				}
			}

			/*
			 * Update the position of each particle in the system using the
			 * Verlet integrator.
			 */
			private void updatePositions(float sx, float sy, long timestamp) {
				final long t = timestamp;
				if (mLastT != 0) {
					final float dT = (float) (t - mLastT)
							* (1.0f / 2500000000.0f);
					if (mLastDeltaT != 0) {
						final float dTC = dT / mLastDeltaT;
						final int count = mBalls.length;
						for (int i = 0; i < count && !pause; i++) {
							mBalls[i].computePhysics(sx, sy, dT, dTC);
						}
					}
					mLastDeltaT = dT;
				}
				mLastT = t;
			}

			/*
			 * Performs one iteration of the simulation. First updating the
			 * position of all the particles and resolving the constraints and
			 * collisions.
			 */
			public void update(float sx, float sy, long now) {
				// update the system's positions
				updatePositions(sx, sy, now);

				// We do no more than a limited number of iterations
				final int NUM_MAX_ITERATIONS = 10;

				/*
				 * Resolve collisions, each particle is tested against every
				 * other particle for collision. If a collision is detected the
				 * particle is moved away using a virtual spring of infinite
				 * stiffness.
				 */
				boolean more = true;
				final int count = mBalls.length;
				for (int k = 0; k < NUM_MAX_ITERATIONS && more; k++) {
					more = false;
					for (int i = 0; i < count; i++) {
						Particle curr = mBalls[i];
						for (int j = i + 1; j < count; j++) {
							Particle ball = mBalls[j];
							float dx = ball.mPosX - curr.mPosX;
							float dy = ball.mPosY - curr.mPosY;
							float dd = dx * dx + dy * dy;
							// Check for collisions
							if (dd <= sBallDiameter2) {
								/*
								 * add a little bit of entropy, after nothing is
								 * perfect in the universe.
								 */
								dx += ((float) Math.random() - 0.5f) * 0.0001f;
								dy += ((float) Math.random() - 0.5f) * 0.0001f;
								dd = dx * dx + dy * dy;
								// simulate the spring
								final float d = (float) Math.sqrt(dd);
								final float c = (0.5f * (sBallDiameter - d))
										/ d;
								curr.mPosX -= dx * c;
								curr.mPosY -= dy * c;
								ball.mPosX += dx * c;
								ball.mPosY += dy * c;
								more = true;
							}
						}
						/*
						 * Finally make sure the particle doesn't intersects
						 * with the walls.
						 */
						curr.resolveCollisionWithBounds();
					}
				}
			}

			public int getParticleCount() {
				return mBalls.length;
			}

			public float getPosX(int i) {
				return mBalls[i].mPosX;
			}

			public float getPosY(int i) {
				return mBalls[i].mPosY;
			}

			public int getBoxX(int i) {
				return mBalls[i].mBoxX;
			}

			public int getBoxY(int i) {
				return mBalls[i].mBoxY;
			}
		}

		public void startSimulation() {
			/*
			 * It is not necessary to get accelerometer events at a very high
			 * rate, by using a slower rate (SENSOR_DELAY_UI), we get an
			 * automatic low-pass filter, which "extracts" the gravity component
			 * of the acceleration. As an added benefit, we use less power and
			 * CPU resources.
			 */
			mSensorManager.registerListener(this, mAccelerometer,
					(SensorManager.SENSOR_DELAY_UI));
		}


		public int getBoxXFromMeter(float i) {// get box from a position
			return (int) (i *xs / boxWidth);
		}

		public int getBoxYFromMeter(float i) {// get box from a position
			return (int) (i *ys/ boxHeight);
		}

		public int getBoxXFromPixel(float i) {// get box from a position
			return (int) (i / boxWidth);
		}

		public int getBoxYFromPixel(float i) {// get box from a position
			return (int) (i / boxHeight);
		}

		public void stopSimulation() {
			mSensorManager.unregisterListener(this);
		}

		public SimulationView(Context context, int CcX, int CcY) {
			super(context);
			CellCountX = Math.max(Math.max(CcX, 2),CcY/2);
			CellCountY = Math.max(Math.max(CcY, 2),CcX/2);
			TrapCount = (int)(CellCountX*CellCountY/7f);
			mAccelerometer = mSensorManager
					.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

			metrics = new DisplayMetrics();
			getWindowManager().getDefaultDisplay().getMetrics(metrics);
			mXDpi = metrics.xdpi;
			mYDpi = metrics.ydpi;
			mMetersToPixelsX = mXDpi / 0.0254f;
			mMetersToPixelsY = mYDpi / 0.0254f;

			// Create graphics array
			boxHeight = (metrics.heightPixels / CellCountY);
			boxWidth = (metrics.widthPixels / CellCountX);

			Boxes = new Box[CellCountX][CellCountY];

			GenerateMaze2(CellCountX,CellCountY, TrapCount, Boxes);

			int ballHeight = (int) (sBallDiameter * mMetersToPixelsY + .5f);
			int ballWidth = (int) (sBallDiameter * mMetersToPixelsX + .5f);

			// rescale the ball so it's about 0.5 cm on screen
			Bitmap ball = BitmapFactory.decodeResource(getResources(),R.drawable.ball);
			mBitmap = Bitmap.createScaledBitmap(ball, ballWidth, ballHeight,true);

			Options opts = new Options();
			opts.inDither = true;
			opts.inPreferredConfig = Bitmap.Config.RGB_565;
			mWood = BitmapFactory.decodeResource(getResources(),R.drawable.wood, opts);
			mWood = Bitmap.createScaledBitmap(mWood, metrics.widthPixels,metrics.heightPixels, true);

			line = new Paint();
			line.setColor(Color.YELLOW);
			line.setStrokeWidth(3);

			line2 = new Paint();
			line2.setColor(Color.CYAN);
			line2.setStrokeWidth(3);

			TrapPaint = new Paint();
			TrapPaint.setColor(Color.BLACK);
		}

		@Override
		protected void onSizeChanged(int w, int h, int oldw, int oldh) {
			// compute the origin of the screen relative to the origin of
			// the bitmap
			mXOrigin = (w - mBitmap.getWidth()) * 0.5f;
			mYOrigin = (h - mBitmap.getHeight()) * 0.5f;
			mHorizontalBound = ((w / mMetersToPixelsX - sBallDiameter) * 0.5f);
			mVerticalBound = ((h / mMetersToPixelsY - sBallDiameter) * 0.5f);
			xc = mXOrigin;
			yc = mYOrigin;
			xs = mMetersToPixelsX;
			ys = mMetersToPixelsY;
		}

		@Override
		public void onSensorChanged(SensorEvent event) {
			if (event.sensor.getType() != Sensor.TYPE_ACCELEROMETER)
				return;
			/*
			 * record the accelerometer data, the event's timestamp as well as
			 * the current time. The latter is needed so we can calculate the
			 * "present" time during rendering. In this application, we need to
			 * take into account how the screen is rotated with respect to the
			 * sensors (which always return data in a coordinate space aligned
			 * to with the screen in its native orientation).
			 */

			switch (mDisplay.getRotation()) {
			case Surface.ROTATION_0:
				mSensorX = event.values[0];
				mSensorY = event.values[1];
				break;
			case Surface.ROTATION_90:
				mSensorX = -event.values[1];
				mSensorY = event.values[0];
				break;
			case Surface.ROTATION_180:
				mSensorX = -event.values[0];
				mSensorY = -event.values[1];
				break;
			case Surface.ROTATION_270:
				mSensorX = event.values[1];
				mSensorY = -event.values[0];
				break;
			}

			mSensorTimeStamp = event.timestamp;
			mCpuTimeStamp = System.nanoTime();
		}

		@Override
		protected void onDraw(Canvas canvas) {
			/*
			 * draw the background
			 */

			canvas.drawBitmap(mWood, 0, 0, null);

			// Draw Walls
			float boxWidth = this.boxWidth;
			float boxHeight = this.boxHeight;
			for (int i = 0; i < Boxes.length; i++) {
				for (int j = 0; j < Boxes[0].length; j++) {
					if (!Boxes[i][j].hasDown)
					{
						canvas.drawLine(
								boxWidth*i,boxHeight*(j+1),
								boxWidth*(i+1),boxHeight*(j+1),line
								);
					}
					if (!Boxes[i][j].hasRight){
						canvas.drawLine(
								boxWidth*(i+1),boxHeight*j,
								boxWidth*(i+1),boxHeight*(j+1),line
								);
					}
					//Traps!
					if(Boxes[i][j].isTrap){
						canvas.drawCircle((i+.5f)*boxWidth, (j+.5f)*boxHeight, Math.min(boxWidth/2, boxHeight/2)-2, TrapPaint);
					}
				}
			}
			// Draw Borders
			canvas.drawLine(0, 2, metrics.widthPixels, 2, line);
			canvas.drawLine(2, 0, 2, metrics.heightPixels, line);
			canvas.drawLine(0, metrics.heightPixels - 2, metrics.widthPixels,
					metrics.heightPixels - 2, line);
			canvas.drawLine(metrics.widthPixels - 2, 0,
					metrics.widthPixels - 2, metrics.heightPixels, line);

			//Start and end text.
			canvas.drawText("START", boxWidth/2, boxHeight/2, line);
			canvas.drawText("END!!", metrics.widthPixels-boxWidth/2, metrics.heightPixels-boxHeight/2, line);

			/*
			 * compute the new position of our object, based on accelerometer
			 * data and present time.
			 */

			final ParticleSystem particleSystem = mParticleSystem;
			final long now = mSensorTimeStamp
					+ (System.nanoTime() - mCpuTimeStamp);
			final float sx = mSensorX;
			final float sy = mSensorY;

			particleSystem.update(sx, sy, now);

			final Bitmap bitmap = mBitmap;
			final int count = particleSystem.getParticleCount();
			for (int i = 0; i < count; i++) {
				/*
				 * We transform the canvas so that the coordinate system matches
				 * the sensors coordinate system with the origin in the center
				 * of the screen and the unit is the meter.
				 */

				final float x = xc + particleSystem.getPosX(i) * xs;
				final float y = yc - particleSystem.getPosY(i) * ys;
				canvas.drawBitmap(bitmap, x, y, null);

				/*canvas.drawText("("
					+ ((Integer)((Float)x).intValue()).toString()+ ","
					+ ((Integer)((Float)y).intValue()).toString() + ") ("
					+ ((Integer) particleSystem.getBoxX(i)).toString()+ ","
					+ ((Integer) particleSystem.getBoxY(i)).toString()+ ")",
					x, y, line);*/

			}

			// and make sure to redraw asap
			invalidate();
		}

		@Override
		public void onAccuracyChanged(Sensor sensor, int accuracy) {
		}
	}

	public class Box{
		public boolean visited;
		public boolean isGoal;
		public boolean hasLeft;
		public boolean hasRight;
		public boolean hasUp;
		public boolean hasDown;
		public boolean isTrap;
		public Box Left;
		public Box Right;
		public Box Up;
		public Box Down;
		public int numNeighbors;
		int x,y;

		public Box(int i, int j){
			visited = false;
			isGoal = false;
			isTrap=false;
			x=i;
			y=j;
		}
		public void addLeftNeighbor(Box i){
			Left = i;
			hasLeft = (i!=null);
		}
		public void addRightNeighbor(Box i){
			Right = i;
			hasRight = (i!=null);
		}
		public void addUpNeighbor(Box i){
			Up = i;
			hasUp = (i!=null);
		}
		public void addDownNeighbor(Box i){
			Down = i;
			hasDown = (i!=null);
		}
		public void addNeighbor(Box i){//Please don't pass incorrect boxes.
			if(x>i.x){addLeftNeighbor(i);}
			else if(x<i.x){addRightNeighbor(i);
			}else if (y<i.y){addDownNeighbor(i);}
			else{addUpNeighbor(i);}
		}

		public boolean Validate() {
			visited = true;
			return !isTrap && 
					(isGoal || 
					(hasLeft && !Left.visited && !Left.isTrap && Left.Validate()) || 
					(hasRight && !Right.visited && !Right.isTrap && Right.Validate())||
					(hasUp && !Up.visited && !Up.isTrap && Up.Validate()) ||
					(hasDown && !Down.visited && !Down.isTrap && Down.Validate()));		
		}
		public Box getRandomUnvisitedNeighbor(Box[][] Boxes, int CellCountX, int CellCountY){
			int count = 0;
			Box[] Neighbors = new Box[4];
			if(!hasLeft && x>0 &&!Boxes[x-1][y].visited && !Boxes[x-1][y].isTrap){
				Neighbors[count++] = Boxes[x-1][y];}
			if(!hasRight && x<CellCountX-1 && !Boxes[x+1][y].visited && !Boxes[x+1][y].isTrap){
				Neighbors[count++] = Boxes[x+1][y];}
			if(!hasUp && y>0 && !Boxes[x][y-1].visited && !Boxes[x][y-1].isTrap){
				Neighbors[count++] = Boxes[x][y-1];}
			if(!hasDown && y<CellCountY-1 && !Boxes[x][y+1].visited && !Boxes[x][y+1].isTrap){
				Neighbors[count++] = Boxes[x][y+1];}
			if(count==0){Neighbors[0] = new Box(-1,-1);}
			return Neighbors[(int)(Math.random()*count)];
		}
	}



	public void GenerateMaze2(int CellCountX, int CellCountY, int TrapCount, Box[][] Boxes ) {
		//Make traps!
		boolean valid;
		do{
			valid = false;
			for (int i = 0; i < CellCountX; i++) {
				for (int j = 0; j < CellCountY; j++) {
					Boxes[i][j]=new Box(i,j);
				}
			}
			Boxes[CellCountX-1][CellCountY-1].isGoal = true;
			for(int i = 0; i < TrapCount; ++i){
				boolean again;
				do{
					int x = (int)(Math.random()*CellCountX);
					int y = (int)(Math.random()*CellCountY);
					again = (x==0&&y==0)||
							(x == CellCountX-1 && y == CellCountY-1) && !Boxes[x][y].isTrap ||
							(x>0 && Boxes[x-1][y].isTrap) ||
							(x<CellCountX-1 && Boxes[x+1][y].isTrap) ||
							(y>0 && Boxes[x][y-1].isTrap) ||
							(y<CellCountY-1 && Boxes[x][y+1].isTrap);
					if (!again){
						Boxes[x][y].isTrap=true;
						if(x>0){
							Boxes[x][y].addNeighbor(Boxes[x-1][y]);
							Boxes[x-1][y].addNeighbor(Boxes[x][y]);
						}if(x<CellCountX-1){
							Boxes[x][y].addNeighbor(Boxes[x+1][y]);
							Boxes[x+1][y].addNeighbor(Boxes[x][y]);
						}if(y>0){
							Boxes[x][y].addNeighbor(Boxes[x][y-1]);
							Boxes[x][y-1].addNeighbor(Boxes[x][y]);
						}if(y<CellCountY-1){
							Boxes[x][y].addNeighbor(Boxes[x][y+1]);
							Boxes[x][y+1].addNeighbor(Boxes[x][y]);
						}
					}
				} while(again);
			}
			List<Box> Nodes = new ArrayList<Box>();
			Nodes.add(Boxes[0][0]);
			Boxes[0][0].visited = true;
			while (!Nodes.isEmpty()){
				int i = (int)(Math.random()*Nodes.size());
				Box Node = Nodes.get(i);
				Box NextNode = Node.getRandomUnvisitedNeighbor(Boxes,CellCountX,CellCountY);
				if(NextNode.x==-1){
					Nodes.remove(i);
				}else{
					NextNode.visited = true;
					valid |= NextNode.isGoal;
					Node.addNeighbor(NextNode);
					NextNode.addNeighbor(Node);
					Nodes.add(NextNode);
				}
			}


		}while(/*!ValidateMaze(CellCountX, CellCountY, TrapCount,Boxes)*/!valid);
		int end = 0;
	}
	public void GenerateMaze(int CellCountX, int CellCountY, int TrapCount, Box[][] Boxes ) {
		//Make traps!
		do{
			for (int i = 0; i < CellCountX; i++) {
				for (int j = 0; j < CellCountY; j++) {
					Boxes[i][j]=new Box(i,j);
				}
			}
			for (int i = 0; i < CellCountX; i++) {
				for (int j = 0; j < CellCountY; j++) {
					boolean a = Math.random()>=.5;
					boolean b = Math.random()>=.5;
					if(a && i+1!=CellCountX){
						Boxes[i][j].addRightNeighbor(Boxes[i+1][j]);
						Boxes[i+1][j].addLeftNeighbor(Boxes[i][j]);
					}
					if(b && j+1!=CellCountY){
						Boxes[i][j].addDownNeighbor(Boxes[i][j+1]);
						Boxes[i][j+1].addUpNeighbor(Boxes[i][j]);
					}
				}
			}
			Boxes[CellCountX-1][CellCountY-1].isGoal = true;
			for(int i = 0; i < TrapCount; ++i){
				boolean again;
				do{
					int x = (int)(Math.random()*CellCountX);
					int y = (int)(Math.random()*CellCountY);
					again = (x==0&&y==0)||(x == CellCountX-1 && y == CellCountY-1) && !Boxes[x][y].isTrap;
					if (!again){Boxes[x][y].isTrap=true;}
				} while(again);
			}
		}while(!ValidateMaze(CellCountX, CellCountY, TrapCount,Boxes));
		int end = 0;
	}
	public boolean ValidateMaze(int CellCountX, int CellCountY, int TrapCount, Box[][] Boxes){
		boolean valid = Boxes[0][0].Validate();
		//return valid;
		return true;
	}
}