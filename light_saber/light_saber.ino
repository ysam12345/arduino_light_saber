    if(apMotion->IsClash())
    {
      Serial.print("Clash detected:");
      switch(apMotion->GetClashMagnitude())
      {
      case eeSmall:
        Serial.println("Small");
        break;
      case eeMedium:
        Serial.println("Medium");
        break;
      case eeLarge:
        Serial.println("Large");
        if(clash_count == 0){
          clash_count++;
          lastTime = millis();
          myDFPlayer.play(random(3)+7); 
          for(int i = 0 ; i < 6 ; i++ ){
            led_light[i] = 255;
          }
          set_light();
          delay(30);
          for(int i = 0 ; i < 6 ; i++ ){
            led_light[i] = 0;
          }
          fade_amount = 10;
          set_light();
          delay(350);
          
          myDFPlayer.loop(1);
        }
        clash_count++;
        break;
      default:
        Serial.println("Unknown");
        break;
      }
    }
