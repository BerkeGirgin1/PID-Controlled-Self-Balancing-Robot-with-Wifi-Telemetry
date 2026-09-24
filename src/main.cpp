#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <WiFi.h>
#include <WiFiUdp.h>


MPU6050 mpu;
WiFiUDP udp;

// KABLOSUZ HABERLEŞME AYARLARI

const char *ssid = "Denge_Robotu_WIFI";
const char *password = "12345678"; 
const int udpPort = 8080;

// Sürüş Komutları
volatile float user_throttle = 0.0f; // İleri/Geri
volatile float user_steering = 0.0f; // Sağ/Sol

// PİN TANIMLAMALARI
// ==========================================
#define SDA_PIN    21
#define SCL_PIN    22

#define STEP_PIN_L 25
#define DIR_PIN_L  26
#define STEP_PIN_R 27
#define DIR_PIN_R  14


#define GYRO_SIGN  1  


// DEĞİŞKENLER ve State Machine

enum RobotState { INIT, BALANCING, FAULT };
volatile RobotState state = INIT;

int16_t ax, ay, az, gx, gy, gz;
float robotAngle = 0.0f;
float angleOffset = 0.0f;

const float dt = 0.004f; // 4ms kontrol döngüsü

// --- CASCADE PID KATSAYILARI ---
float Kp_st  = 3.5f;
float Kd_st  = 0.13f;
float Kp_thr = 0.016f;
float Ki_thr = 0.1;

// Cascade Sistemin Hafıza Elemanları
float target_angle             = 0.0f;
float speed_integral           = 0.0f;
float estimated_speed_filtered = 0.0f;
float acceleration             = 0.0f;

volatile float currentSpeed  = 0.0f;
volatile int   speed_L       = 0;
volatile int   speed_R       = 0;
volatile int   counter_L     = 0;
volatile int   counter_R     = 0;
const    int   TIMER_PERIOD  = 20;  // 50 kHz = 20us
volatile bool  toggle_L      = false;
volatile bool  toggle_R      = false;
volatile float batteryVoltage = 0.0f;
int faultCounter = 0;

hw_timer_t   *timer    = NULL;
portMUX_TYPE  timerMux = portMUX_INITIALIZER_UNLOCKED;


// TIMER KESMESİ SABİT 50 KHz 

void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    if (state == BALANCING) {
        if (speed_L != 0) {
            counter_L += abs(speed_L);
            if (counter_L >= 2000) {
                toggle_L = !toggle_L;
                digitalWrite(STEP_PIN_L, toggle_L);
                counter_L -= 2000;
            }
        }
        if (speed_R != 0) {
            counter_R += abs(speed_R);
            if (counter_R >= 2000) {
                toggle_R = !toggle_R;
                digitalWrite(STEP_PIN_R, toggle_R);
                counter_R -= 2000;
            }
        }
    }
    portEXIT_CRITICAL_ISR(&timerMux);
}


// CORE 1: PID

void BalanceTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(4);

    float gyroHistory[5] = {0, 0, 0, 0, 0};
    float accHistory[5]  = {0, 0, 0, 0, 0};


    for (;;) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        float ham_accAngle = atan2f((float)az, (float)ax) * 57.296f;

        // --- RUNTIME GYRO BIAS TAKIBI (Sıcaklık Drift Koruması) ---
        static float gyro_bias_runtime = 0.0f;
        static int   bias_settle_count = 0;

        float accel_magnitude = sqrtf((float)ax*(float)ax +
                                      (float)ay*(float)ay +
                                      (float)az*(float)az);
        // Vektör büyüklüğü eksen orientasyonundan bağımsız — değişmedi.
        // 2g scale: 1g ≈ 16384 LSB → ±2000 tolerans vibrasyon için
        bool robot_is_still = (fabsf(robotAngle - angleOffset) < 1.2f) &&
                              (fabsf(currentSpeed)             < 80.0f) &&
                              (accel_magnitude > 14000.0f && accel_magnitude < 18500.0f);

        if (robot_is_still) {
            bias_settle_count++;
            float alpha = (bias_settle_count < 50) ? 0.01f : 0.0005f;
            gyro_bias_runtime = gyro_bias_runtime * (1.0f - alpha) + (float)gy * alpha;
        }

      
        float ham_gyroRate = GYRO_SIGN * ((float)gy - gyro_bias_runtime) / 131.0f;

        // Moving Average DSP Filtresi (5 eleman)
        for (int i = 4; i > 0; i--) {
            gyroHistory[i] = gyroHistory[i-1];
            accHistory[i]  = accHistory[i-1];
        }
        gyroHistory[0] = ham_gyroRate;
        accHistory[0]  = ham_accAngle;

        float temiz_gyroRate = (gyroHistory[0]+gyroHistory[1]+gyroHistory[2]+
                                gyroHistory[3]+gyroHistory[4]) / 5.0f;
        float temiz_accAngle = (accHistory[0] +accHistory[1] +accHistory[2] +
                                accHistory[3] +accHistory[4])  / 5.0f;

        // Complemantary Filtre 
        robotAngle = 0.99f * (robotAngle + temiz_gyroRate * dt) + 0.01f * temiz_accAngle;

        
        if (state == FAULT || state == INIT) {
            if (fabsf(robotAngle - angleOffset) < 2.0f) {
                speed_integral = 0.0f;
                currentSpeed   = 0.0f;
                user_throttle  = 0.0f; // KALKARKEN KOMUTLARI SIFIRLA
                user_steering  = 0.0f; 
                state = BALANCING;
            }
        }

        if (state == BALANCING) {
            if (fabsf(robotAngle) > 40.0f) {
                faultCounter++;
                if (faultCounter > 25) {
                    state = FAULT;
                    speed_L        = 0;
                    speed_R        = 0;
                    speed_integral = 0.0f;
                    currentSpeed   = 0.0f;
                }
            } else {
                faultCounter = 0;

                //  Dış Döngü Hız Filtresi
                float estimated_speed    = -currentSpeed + (temiz_gyroRate * 2.0f);
                estimated_speed_filtered = estimated_speed_filtered * 0.9f + estimated_speed * 0.1f;

                static float smoothed_throttle = 0.0f;
                static float smoothed_steering = 0.0f;
                
                if (user_throttle == 0.0f) {
                  
                    // 1. Hedef hızı çok sert bir şekilde sıfıra çeker
                    smoothed_throttle = smoothed_throttle * 0.70f; 
                    
                    // 2.İleri-geri salınımı yok etmek için İntegral hafızasını sildim
                    speed_integral = speed_integral * 0.85f; 
                } else {
                    // Kullanıcı surerken: İvmelenme devam etsin
                    smoothed_throttle = (smoothed_throttle * 0.985f) + (user_throttle * 0.015f);
                }
                
                smoothed_steering = (smoothed_steering * 0.900f) + (user_steering * 0.100f);

                //  DIŞ DÖNGÜ: HIZ PI 
                float target_speed = smoothed_throttle * 3.0f; 
                float speed_error = target_speed - estimated_speed_filtered; 
                
                speed_integral   += speed_error * dt;
                speed_integral    = constrain(speed_integral, -1000.0f, 1000.0f);
                
                float pi_output = (Kp_thr * speed_error) + (Ki_thr * speed_integral);

                // AÇI ENJEKSİYONU VE LİMİTLEME 
                float drive_angle_offset = smoothed_throttle * 0.015f; 
                target_angle = pi_output + drive_angle_offset;
                
                // Limiti yüksek tutuyoruz ki dururken gerekirse anlık geriye sert yatabilsin
                target_angle = constrain(target_angle, -4.5f, 4.5f);

                //  İç Döngü
                float angle_error = target_angle - (robotAngle - angleOffset);
                acceleration      = (Kp_st * angle_error) - (Kd_st * temiz_gyroRate);

                currentSpeed += acceleration;
                currentSpeed  = constrain(currentSpeed, -2000.0f, 2000.0f);

                // DİFERANSİYEL SÜRÜŞ
                float out_L = currentSpeed + smoothed_steering;
                float out_R = currentSpeed - smoothed_steering;

                if (out_L >= 0) digitalWrite(DIR_PIN_L, LOW);
                else            digitalWrite(DIR_PIN_L, HIGH);

                if (out_R >= 0) digitalWrite(DIR_PIN_R, LOW);
                else            digitalWrite(DIR_PIN_R, HIGH);

                portENTER_CRITICAL(&timerMux);
                speed_L = (int)fabsf(out_L);
                speed_R = (int)fabsf(out_R);
                portEXIT_CRITICAL(&timerMux);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


// CORE 0: TELEMETRİ 

void TelemetryTask(void *pvParameters) {
    char packetBuffer[64]; 

    for (;;) {
      
        // WI-FI 
        
        int packetSize;
        while ((packetSize = udp.parsePacket()) > 0) {
            int len = udp.read(packetBuffer, 63);
            if (len > 0) {
                packetBuffer[len] = '\0';}

            char komut = ' ';
            float deger = 0.0f;

            if (sscanf(packetBuffer, " %c %f", &komut, &deger) >= 1) {
                // PID KATSAYI KOMUTLARI
                if      (komut == 'P' || komut == 'p') Kp_st = deger;
                else if (komut == 'D' || komut == 'd') Kd_st = deger;
                else if (komut == 'T' || komut == 't') Kp_thr = deger;
                else if (komut == 'I' || komut == 'i') Ki_thr = deger;
                else if (komut == 'O' || komut == 'o') {
                    angleOffset              = deger;
                    speed_integral           = 0.0f;
                    estimated_speed_filtered = 0.0f;
                }
                else if (komut == 'R' || komut == 'r') {
                    state = FAULT;
                    portENTER_CRITICAL(&timerMux);
                    speed_L = 0; speed_R = 0;
                    portEXIT_CRITICAL(&timerMux);
                    speed_integral = 0.0f; currentSpeed = 0.0f;
                    estimated_speed_filtered = 0.0f; faultCounter = 0;
                    state = INIT;
                }
                // SÜRÜŞ KOMUTLARI
                else if (komut == 'Y' || komut == 'y') {
                    user_throttle = deger; 
                }
                else if (komut == 'X' || komut == 'x') {
                    user_steering = deger; 
                }
            }
        }

        
        // TELEMETRİ GÖNDERİMİ 
        
        IPAddress broadcastIP(255, 255, 255, 255);
        udp.beginPacket(broadcastIP, udpPort);
        udp.printf("START,%.2f,%.2f,%.2f,%.2f,%d,END\n", 
                   robotAngle, target_angle, acceleration, currentSpeed, state);
        udp.endPacket();

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// KALİBRASYON FONKSİYONU

void calibrateMPU_onBoot() {
    int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;

    Serial.println("MPU osilator settle bekleniyor (1500ms)...");
    delay(1500);

    // Registerları temiz başlattım
    mpu.setXAccelOffset(0); mpu.setYAccelOffset(0); mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);  mpu.setYGyroOffset(0);  mpu.setZGyroOffset(0);
    delay(100);

    // DLPF pipeline'ını doldur (50 ısınma okuması)
    for (int i = 0; i < 50; i++) {
        mpu.getMotion6(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);
        delay(10);
    }

    // Robot dik konuma gelene kadar bekler
    Serial.println("Robotu DIK konuma getirin (20 derece tolerans)...");
    while (true) {
        mpu.getMotion6(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);
        
        float a = atan2f((float)az_r, (float)ax_r) * 57.296f;
        if (fabsf(a) < 20.0f) {
            Serial.println("Dik konum algilandi! HAREKETSIZ TUTUN...");
            delay(1000);
            break;
        }
        delay(50);
    }

    bool cal_ok = false;
    while (!cal_ok) {
        long sum_ax=0, sum_ay=0, sum_az=0;
        long sum_gx=0, sum_gy=0, sum_gz=0;
        int  valid_n = 0;
        const int N  = 300;

        Serial.println("Kalibrasyon basliyor (300 sample, 20 derece bant)...");

        for (int i = 0; i < N; i++) {
            mpu.getMotion6(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);
            
            float instant_angle = atan2f((float)az_r, (float)ax_r) * 57.296f;
            if (i % 30 == 0) {
                Serial.printf("[300 DONGUSU ICI] ax: %d | az: %d | Hesaplanan: %.2f\n", ax_r, az_r, instant_angle);
            }
            if (fabsf(instant_angle) < 20.0f) {
                sum_ax += ax_r; sum_ay += ay_r; sum_az += az_r;
                sum_gx += gx_r; sum_gy += gy_r; sum_gz += gz_r;
                valid_n++;
            }
            delay(4);
        }

        if (valid_n < 200) {
            Serial.print("Gecersiz sample sayisi: "); Serial.print(N - valid_n);
            Serial.println(" — Robot titredı, tekrar...");
            delay(1000);
            continue;
        }

        float m_ax = (float)sum_ax / valid_n;
        float m_ay = (float)sum_ay / valid_n;
        float m_az = (float)sum_az / valid_n;
        float m_gx = (float)sum_gx / valid_n;
        float m_gy = (float)sum_gy / valid_n;
        float m_gz = (float)sum_gz / valid_n;

        
        mpu.setXAccelOffset((int16_t)(-(m_ax - 16384.0f) / 8.0f));
        mpu.setYAccelOffset((int16_t)(-m_ay               / 8.0f));
        mpu.setZAccelOffset((int16_t)(-m_az               / 8.0f));
        mpu.setXGyroOffset ((int16_t)(-m_gx               / 4.0f));
        mpu.setYGyroOffset ((int16_t)(-m_gy               / 4.0f));
        mpu.setZGyroOffset ((int16_t)(-m_gz               / 4.0f));

        delay(150);  // Register yazımı settle

        // Başlangıç açısını 20 örnek ortalamasıyla hesapla
        float angle_sum = 0.0f;
        int   angle_n   = 0;
        for (int i = 0; i < 20; i++) {
            mpu.getMotion6(&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);
            float a = atan2f((float)az_r, (float)ax_r) * 57.296f;
            if (fabsf(a) < 20.0f) { angle_sum += a; angle_n++; }
            delay(5);
        }

        if (angle_n < 10) {
            Serial.println("Baslangic acisi gecersiz (az sample), tekrar...");
            delay(500);
            continue;
        }

        robotAngle = angle_sum / angle_n;

        if (fabsf(robotAngle) < 20.0f) {
            cal_ok = true;
        } else {
            Serial.print("Kalibrasyon acisi cok buyuk: ");
            Serial.print(robotAngle, 2);
            Serial.println(" — Tekrar...");
            delay(500);
        }
    }

    // Kalibrasyon sonrası robotun bulunduğu açıyı sıfır noktası yaptım
  
    angleOffset              = robotAngle;
    speed_integral           = 0.0f;
    currentSpeed             = 0.0f;
    estimated_speed_filtered = 0.0f;
    faultCounter             = 0;

    Serial.print("Kalibrasyon OK! Baslangic acisi: ");
    Serial.print(robotAngle, 3);
    Serial.print("  angleOffset: ");
    Serial.println(angleOffset, 3);
}


// SETUP

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(50);         
   

    WiFi.softAP(ssid, password);
    udp.begin(udpPort);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);          // Fast-mode I2C — kararlılık ve hız için


    pinMode(STEP_PIN_L, OUTPUT); pinMode(DIR_PIN_L, OUTPUT);
    pinMode(STEP_PIN_R, OUTPUT); pinMode(DIR_PIN_R, OUTPUT);

    timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, TIMER_PERIOD, true, 0);

    // mpu.initialize() SADECE BİR KEZ — calibrateMPU_onBoot içinde YOK
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("[HATA] MPU6050 bulunamadi! I2C kontrolu yap.");
        while (1) delay(500);
    }
    Serial.println("[OK] MPU6050 baglantisi dogrulandi.");

    calibrateMPU_onBoot();

    xTaskCreatePinnedToCore(BalanceTask,    "Balance",   4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(TelemetryTask, "Telemetry", 4096, NULL, 1, NULL, 0);
}

void loop() {
    // Boş. tüm yük FreeRTOS çekirdeklerinde
}
