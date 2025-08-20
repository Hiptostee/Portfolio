import React, {useState, useEffect} from 'react';
import axios from 'axios';

import {
  ScrollView,
  StatusBar,
  View,
  TextInput,
  Text,
  ActivityIndicator,
} from 'react-native';
import {
  GoogleSignin,
  GoogleSigninButton,
  statusCodes,
} from 'react-native-google-signin';
import Config from 'react-native-config';
interface LoggedInScreenProps {
  navigation: any;
}
function LoginController({navigation}: LoggedInScreenProps) {
  const [isSigninInProgress, setIsSigninInProgress] = useState(false);
  const [studentId, setStudentId] = useState('');
  const [showInputMessage, setShowInputMessage] = useState(false);
  const [loadingVerification, setLoadingVerification] = useState(false);
  const [showUnvalidatedMessage, setShowUnvalidatedMessage] = useState(false);

  useEffect(() => {
    GoogleSignin.configure({
      webClientId: Config.GOOGLE_WEB_CLIENT_ID,
      offlineAccess: true,
      hostedDomain: '',
      forceConsentPrompt: true,
    });
    getCurrentUserInfo(); //
  });

  const getCurrentUserInfo = async () => {
    try {
      const userInfo = await GoogleSignin.signInSilently();
      setIsSigninInProgress(false);
      if (userInfo.user.email === 'joey.marra2007@gmail.com') {
        navigation.navigate('TeacherDashboard');
        setLoadingVerification(false);
        setShowInputMessage(false);
        setShowUnvalidatedMessage(false);
      } else {
        navigation.navigate('Dashboard', {id: studentId});
        setLoadingVerification(false);
        setShowInputMessage(false);
        setShowUnvalidatedMessage(false);
      }
    } catch (error: any) {
      if (error.code === statusCodes.SIGN_IN_REQUIRED) {
      } else {
      }
      setIsSigninInProgress(false);
    }
  };

  const verifyUser = async (email: string) => {
    try {
      const data = await axios
        .get(
          `https://id.jaybots.org/api/getUsers?email=${email}&studentId=${studentId}`,
        )
        .then(response => response.data);
      return data;
    } catch (error) {
      console.error('Verification error:', error);
      return false; 
    }
  };
  const signOut = async () => {
    try {
      await GoogleSignin.revokeAccess();
      await GoogleSignin.signOut();
      navigation.navigate('Home');
    } catch (error) {
      console.error('Error signing out: ', error);
    }
  };

  const signIn = async () => {
    if (studentId.trim() === '') {
      setShowInputMessage(true);
      return;
    }
    setShowUnvalidatedMessage(false);

    setLoadingVerification(true);

    try {
      await GoogleSignin.hasPlayServices();
      const user = await GoogleSignin.signIn();

      setIsSigninInProgress(false);
      const verified = await verifyUser(user.user.email);
      if (!verified) {
        setShowUnvalidatedMessage(true);
        setLoadingVerification(false);
        signOut();
        console.log('it failed');
        return;
      }
      if (user.user.email === 'joey.marra2007@gmail.com') {
        navigation.navigate('TeacherDashboard');
        setLoadingVerification(false);
        setShowInputMessage(false);
        setShowUnvalidatedMessage(false);
      } else {
        navigation.navigate('Dashboard', {id: studentId});
        setLoadingVerification(false);
        setShowInputMessage(false);
        setShowUnvalidatedMessage(false);
      }
    } catch (error: any) {
      setLoadingVerification(false);
      setIsSigninInProgress(false);
    }
  };

  return (
    <View className="mb-16 text-white">
      <StatusBar />
      <ScrollView contentInsetAdjustmentBehavior="automatic">
        <View>
          <TextInput
            className="bg-gray-800 border-2 border-gray-800 rounded-lg px-4 py-2 w-80 mx-auto mt-8 text-white"
            placeholder="Enter Student ID"
            placeholderTextColor="#9ca3af" 
            value={studentId}
            onChangeText={text => {
              setStudentId(text);
              setShowInputMessage(false);
            }}
          />
          {showInputMessage && (
            <Text className="text-red-500 mx-auto mt-4">
              Please input Student ID to continue.
            </Text>
          )}
          {showUnvalidatedMessage && (
            <Text className="text-red-500 mx-auto mt-4">
              Your account could not be verified.
            </Text>
          )}

          <View className=" mt-8 w-full mx-auto">
            {loadingVerification ? (
              <ActivityIndicator />
            ) : (
              <GoogleSigninButton
                style={{width: 192, height: 48}}
                size={GoogleSigninButton.Size.Wide}
                color={GoogleSigninButton.Color.Dark}
                onPress={signIn}
                className=" mx-auto"
                disabled={isSigninInProgress}
              />
            )}
          </View>
        </View>
      </ScrollView>
    </View>
  );
}

export default LoginController;
