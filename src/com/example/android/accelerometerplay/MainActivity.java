package com.example.android.accelerometerplay;

import android.os.Bundle;
import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.view.Menu;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

public class MainActivity extends Activity {

	Context context = this;
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		
		Button button = (Button) findViewById(R.id.button1);
		button.setOnClickListener(new OnClickListener() {
			public void onClick(View arg0){
				TextView textRows = (TextView) findViewById(R.id.editText2);
				TextView textColumns = (TextView) findViewById(R.id.editText1);
				Intent levelDown = new Intent(context, AccelerometerPlayActivity.class);
				Bundle parem = new Bundle();
				parem.putInt("level", 1);
				parem.putBoolean("AlarmMode", true);
				int CcX,CcY;
				
				try{CcX = Integer.parseInt(textRows.getText().toString());
				}catch (Exception e){CcX = 15; textRows.setText(((Integer)CcX).toString());}
				
				try{CcY = Integer.parseInt(textColumns.getText().toString());
				}catch (Exception e){CcY = 10; textColumns.setText(((Integer)CcY).toString());}
				
				parem.putInt("CcX", Math.min(Math.max(CcX,2),30));
				parem.putInt("CcY", Math.min(Math.max(CcY,2),40));
				levelDown.putExtras(parem);
				((Activity)context).startActivityForResult(levelDown, 1);			}
			
		});
		
		Button buttonAlarm = (Button) findViewById(R.id.button3);
		buttonAlarm.setOnClickListener(new OnClickListener() {
			public void onClick(View arg0){
				TextView textRows = (TextView) findViewById(R.id.editText2);
				TextView textColumns = (TextView) findViewById(R.id.editText1);
				Intent levelDown = new Intent(context, AccelerometerPlayActivity.class);
				Bundle parem = new Bundle();
				parem.putInt("level", 1);
				parem.putBoolean("AlarmMode", true);
				int CcX,CcY;
				
				try{CcX = Integer.parseInt(textRows.getText().toString());
				}catch (Exception e){CcX = 15; textRows.setText(((Integer)CcX).toString());}
				
				try{CcY = Integer.parseInt(textColumns.getText().toString());
				}catch (Exception e){CcY = 10; textColumns.setText(((Integer)CcY).toString());}
				
				parem.putInt("CcX", Math.min(Math.max(CcX,2),30));
				parem.putInt("CcY", Math.min(Math.max(CcY,2),40));
				levelDown.putExtras(parem);
				((Activity)context).startActivityForResult(levelDown, 1);			}
			
		});
		Button buttonQuit = (Button) findViewById(R.id.button2);
		buttonQuit.setOnClickListener(new OnClickListener() {
			public void onClick(View arg0){finish();}
		});
		
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.activity_main, menu);
		return true;
	}
	
	@Override
	public void onActivityResult(int requestCode, int resultCode, Intent data){
		super.onActivityResult(requestCode, resultCode, data);
	}
}
