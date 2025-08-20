import React, {useEffect, useState} from 'react';
import {
  View,
  Text,
  SafeAreaView,
  TouchableOpacity,
  ScrollView,
  StatusBar,
} from 'react-native';
import {GoogleSignin, statusCodes} from 'react-native-google-signin';
import Config from 'react-native-config';

interface LoggedInScreenProps {
  navigation: any;
}

const LoggedInScreen = ({navigation}: LoggedInScreenProps) => {
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
  const signOut = async () => {
    try {
      await GoogleSignin.revokeAccess();
      await GoogleSignin.signOut();
      navigation.navigate('Home');
    } catch (error) {
      console.error('Error signing out: ', error);
    }
  };
  return (
    <SafeAreaView className="bg-slate-900">
      <StatusBar barStyle={'dark-content'} />
      <ScrollView
        contentInsetAdjustmentBehavior="automatic"
        className=" text-white">
        <View className="bg-gray-900 mt-8 px-2  h-screen dark:bg-black">
          <View>
            <Text className="text-3xl text-center font-bold text-white dark:text-white">
              Hello {userInfo?.user.name}!
            </Text>
            <View>
              <Text className="mt-2 mx-8 text-center mb-8 text-base text-gray-400 dark:text-white">
                Scan your digital ID in your Apple Wallet to check in!
              </Text>
            </View>
          </View>
          <View className="flex flex-row p-4 mt-64">
            <TouchableOpacity
              onPress={() => signOut()}
              className="bg-blue-500 mx-auto text-center text-white py-2 px-4 rounded">
              <Text className="text-white">Logout</Text>
            </TouchableOpacity>
            <TouchableOpacity
              onPress={() => navigation.navigate('TeacherDashboard')}
              className="bg-blue-500 mx-auto text-center text-white py-2 px-4 rounded">
              <Text className="text-white">Teacher Dashboard</Text>
            </TouchableOpacity>
            <TouchableOpacity
              onPress={() => navigation.navigate('Passes')}
              className="bg-blue-500 mx-auto text-center text-white py-2 px-4 rounded">
              <Text className="text-white">Passes</Text>
            </TouchableOpacity>
          </View>
        </View>
      </ScrollView>
    </SafeAreaView>
  );
};

export default LoggedInScreen;
