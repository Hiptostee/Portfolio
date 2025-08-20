/* eslint-disable @typescript-eslint/no-shadow */
/* eslint-disable react-native/no-inline-styles */
import React, {useEffect, useState} from 'react';
import WalletManager from 'react-native-wallet-manager';

import {
  View,
  Text,
  Image,
  SafeAreaView,
  TouchableOpacity,
  ScrollView,
  StatusBar,
} from 'react-native';
import {GoogleSignin, statusCodes} from 'react-native-google-signin';
import Config from 'react-native-config';

interface LoggedInScreenProps {
  navigation: any;
  route: any;
}

const LoggedInScreen = ({route}: LoggedInScreenProps) => {
  let studentId = route.params.id;

  const [userInfo, setUserInfo] = useState<UserInfo | null>(null);
  interface UserInfo {
    user: {
      photo: string | null;
      name: string;
      email: string;
      id: string;
    };
  }

  useEffect(() => {
    GoogleSignin.configure({
      webClientId: Config.GOOGLE_WEB_CLIENT_ID,
      offlineAccess: true,
      hostedDomain: '',
      forceConsentPrompt: true,
    });
    getCurrentUserInfo(); 
  });
  const getCurrentUserInfo = async () => {
    try {
      const userInfo = await GoogleSignin.signInSilently();
      const updatedUserInfo: UserInfo = {
        user: {
          photo: userInfo.user.photo || null,
          name: userInfo.user.name || '',
          email: userInfo.user.email,
          id: userInfo.user.id,
        },
      };
      setUserInfo(updatedUserInfo);
    } catch (error: any) {
      console.log(error)
    }
  };

  const generatePass = async (studentEmail: string) => {
    try {
      const result = await WalletManager.addPassFromUrl(
        `https://id.jaybots.org/api/generatePass?email=${studentEmail}&studentId=${studentId}`,
      );
    } catch (e) {
      console.log(e, 'error');
    }
  };
 

  return (
    <SafeAreaView className="bg-gray-900 " style={{flex: 1}}>
      <StatusBar barStyle={'dark-content'} />
      <ScrollView contentContainerStyle={{flexGrow: 1}}>
        <View style={{flex: 1}}>
          <View style={{flex: 1}}>
            <View className="mt-8">
              <Image
                className="mx-auto"
                style={{width: '60%', height: 50}}
                source={require('../images/logo.png')}
              />
              <Text className="mx-auto text-sm mb-16 first-line:mt-0 text-white font-bold ">
                Streamlining your Classroom
              </Text>
            </View>
            <Text
              className="mt-8"
              style={{
                fontSize: 30,
                fontWeight: 'bold',
                color: 'white',
                textAlign: 'center',
              }}>
              Welcome {userInfo?.user.name}!
            </Text>

            <View style={{margin: 8}}>
              <Text
                style={{
                  fontSize: 20,
                  color: 'rgba(255, 255, 255, 0.6)',
                  textAlign: 'center',
                }}>
                You are now logged in. Click the button below to add your
                student ID to Apple Wallet and begin scanning
              </Text>
            </View>
            <View>
              <TouchableOpacity
                className="mt-8"
                onPress={() => generatePass(userInfo?.user.email || '')}
                style={{alignItems: 'center'}}>
                <Image
                  style={{width: 170.49, height: 52.6}}
                  source={require('../images/applewallet.png')}
                />
              </TouchableOpacity>
            </View>
          </View>
        </View>
        <View className="border border-gray-300 mx-2 border-b-1" />
        <View
          style={{
            alignItems: 'center',
            padding: 20,
          }}>
          <Text style={{fontSize: 18, color: 'white'}}>
            Created by Joe, Matt, Anish, Nash
          </Text>
          <Text style={{fontSize: 13, color: 'white'}}>
            John Jay Senior High School
          </Text>
        </View>
      </ScrollView>
    </SafeAreaView>
  );
};

export default LoggedInScreen;
