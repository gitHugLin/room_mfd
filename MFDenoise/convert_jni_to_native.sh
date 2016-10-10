file=$1

sed -i "s/JNIEXPORT//g" $file    
sed -i "s/JNICALL//g" $file
sed -i "s/JNIEnv \*,//g" $file
sed -i "s/JNIEnv \*env,//g" $file
sed -i "s/JNIEnv \* env,//g" $file
sed -i "s/jobject obj,//g" $file
sed -i "s/jobject obj//g" $file
sed -i "s/jint/int/g" $file
sed -i "s/jlong/long/g" $file
sed -i "s/jboolean/bool/g" $file
sed -i "s/(  /(/g" $file
sed -i "s/( /(/g" $file
