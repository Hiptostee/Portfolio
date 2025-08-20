/* eslint-disable @typescript-eslint/no-shadow */
/* eslint-disable react-native/no-inline-styles */
import React, {useEffect, useState} from 'react';
import WalletManager from 'react-native-wallet-manager';
import Config from 'react-native-config';

import {
  View,
  Text,
  Image,
  SafeAreaView,
  TouchableOpacity,
  ScrollView,
  StatusBar,
  TextInput,
} from 'react-native';
import {GoogleSignin, statusCodes} from 'react-native-google-signin';
interface LoggedInScreenProps {
  navigation: any;
  route: any;
}

const LoggedInScreen = ({route}: LoggedInScreenProps) => {
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
      if (error.code === statusCodes.SIGN_IN_REQUIRED) {
        // User has not signed in yet
      } else {
        // Some other error occurred
      }
    }
  };


  const generateHallPass = async (studentEmail: string) => {
    const options = {hour: '2-digit', minute: '2-digit', hour12: true};
    const time = new Date().toLocaleTimeString(undefined, options);
    try {
      const result = await WalletManager.addPassFromUrl(
        `https://id.jaybots.org/api/generateHallPass?email=${studentEmail}&location=${location}&time=${time}&teacher=${teacher}&from=${from}&password=${password}&duration=${duration}`,
      );
    } catch (e) {
      console.log(e, 'error');
    }
  };

  const [location, setLocation] = useState('');
  const [teacher, setTeacher] = useState('');
  const [from, setFrom] = useState('');
  const [showInputMessage, setShowInputMessage] = useState(false);
  const [password, setPassword] = useState('');
  const [duration, setDuration] = useState('');

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
              <Text className="mx-auto text-sm mb-4 first-line:mt-0 text-white font-bold ">
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
              Create Passes
            </Text>
            <Text
              className="mt-4"
              style={{
                fontSize: 12,
                fontWeight: 'bold',
                color: 'white',
                textAlign: 'center',
              }}>
              Location
            </Text>
            <TextInput
              className="bg-gray-800 border-2 border-gray-800 rounded-lg px-4 py-2 w-80 mx-auto mt-8 text-white"
              placeholder="Enter Location"
              placeholderTextColor="#9ca3af"
              value={location}
              onChangeText={text => {
                setLocation(text);
                setShowInputMessage(false);
              }}
            />
            <Text
              className="mt-4"
              style={{
                fontSize: 12,
                fontWeight: 'bold',
                color: 'white',
                textAlign: 'center',
              }}>
              From
            </Text>
            <TextInput
              className="bg-gray-800 border-2 border-gray-800 rounded-lg px-4 py-2 w-80 mx-auto mt-8 text-white"
              placeholder="Enter Current Location"
              placeholderTextColor="#9ca3af"
              value={from}
              onChangeText={text => {
                setFrom(text);
                setShowInputMessage(false);
              }}
            />
            <Text
              className="mt-4"
              style={{
                fontSize: 12,
                fontWeight: 'bold',
                color: 'white',
                textAlign: 'center',
              }}>
              Duration
            </Text>
            <TextInput
              className="bg-gray-800 border-2 border-gray-800 rounded-lg px-4 py-2 w-80 mx-auto mt-8 text-white"
              placeholder="Enter Current Location"
              placeholderTextColor="#9ca3af" 
              value={duration}
              onChangeText={text => {
                setDuration(text);
                setShowInputMessage(false);
              }}
            />
            <Text
              className="mt-4"
              style={{
                fontSize: 12,
                fontWeight: 'bold',
                color: 'white',
                textAlign: 'center',
              }}>
              Teacher
            </Text>
            <TextInput
              className="bg-gray-800 border-2 border-gray-800 rounded-lg px-4 py-2 w-80 mx-auto mt-8 text-white"
              placeholder="Enter Teacher Name"
              placeholderTextColor="#9ca3af"
              value={teacher}
              onChangeText={text => {
                setTeacher(text);
                setShowInputMessage(false);
              }}
            />
            <Text
              className="mt-4"
              style={{
                fontSize: 12,
                fontWeight: 'bold',
                color: 'white',
                textAlign: 'center',
              }}>
              Password
            </Text>
            <TextInput
              className="bg-gray-800 border-2 border-gray-800 rounded-lg px-4 py-2 w-80 mx-auto mt-8 text-white"
              placeholder="Enter teacher password"
              secureTextEntry={true}
              placeholderTextColor="#9ca3af" 
              value={password}
              onChangeText={text => {
                setPassword(text);
                setShowInputMessage(false);
              }}
            />
            {showInputMessage && (
              <Text className="text-red-500 mx-auto mt-4">
                Please input Student ID to continue.
              </Text>
            )}
            <View>
              <TouchableOpacity
                className="my-8"
                onPress={() => generateHallPass(userInfo?.user.email)}
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
