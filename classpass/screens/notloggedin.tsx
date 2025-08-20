/* eslint-disable react-native/no-inline-styles */
import React from 'react';
import LoginController from './LoginController';
import {
  SafeAreaView,
  ScrollView,
  StatusBar,
  useColorScheme,
  Text,
  View,
  Image,
} from 'react-native';
interface LoggedInScreenProps {
  navigation: any;
  route: any;
}
const HomeScreen = ({navigation}: LoggedInScreenProps) => {
  const isDarkMode = useColorScheme() === 'dark';

  return (
    <SafeAreaView style={{flex: 1}} className="bg-slate-900">
      <StatusBar barStyle={isDarkMode ? 'light-content' : 'dark-content'} />
      <ScrollView contentContainerStyle={{flexGrow: 1}}>
        <View style={{flex: 1}}>
          <View style={{flex: 1}}>
            <View className="mt-8">
              <Image
                style={{width: '100%', height: 50}}
                source={require('../images/logo.png')}
              />
              <Text className="mx-auto mb-36 first-line:mt-2 text-white font-bold ">
                Streamlining your Classroom
              </Text>
            </View>
            <Text className="mt-2 border border-white mx-6 text-center font-bold  text-xl text-gray-100 ">
              Log in with your school Google account to get started!
            </Text>
            <LoginController navigation={navigation} />
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

export default HomeScreen;
