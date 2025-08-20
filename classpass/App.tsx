import HomeScreen from './screens/notloggedin';
import LoggedInScreen from './screens/loggedin';
import Attendance from './screens/Attendance';
import Passes from './screens/Passes';
import TeacherDashboard from './screens/teacherLoggedIn';
import {createNativeStackNavigator} from '@react-navigation/native-stack';
import {NavigationContainer} from '@react-navigation/native';
import {Button} from 'react-native';
import {GoogleSignin} from 'react-native-google-signin';

const Stack = createNativeStackNavigator();

export default function App() {
  const signOut = async (navigation: {navigate: (arg0: string) => void}) => {
    try {
      await GoogleSignin.revokeAccess();
      await GoogleSignin.signOut();
      navigation.navigate('Home');
    } catch (error) {
      console.error('Error signing out: ', error);
    }
  };

  return (
    <NavigationContainer>
      <Stack.Navigator
        screenOptions={{
          headerStyle: {
            backgroundColor: '#1e293b',
          },
          headerTitleStyle: {
            color: '#f3f4f6',
          },
        }}>
        <Stack.Screen name="Home" component={HomeScreen} />
        <Stack.Screen
          options={({navigation}) => ({
            headerLeft: () => (
              <Button onPress={() => signOut(navigation)} title="Sign Out" />
            ),
            headerRight: () => (
              <Button
                onPress={() => navigation.navigate('Passes')}
                title="Passes"
              />
            ),
          })}
          name="Dashboard"
          component={LoggedInScreen}
        />
        <Stack.Screen name="Attendance" component={Attendance} />
        <Stack.Screen name="Passes" component={Passes} />
        <Stack.Screen name="TeacherDashboard" component={TeacherDashboard} />
      </Stack.Navigator>
    </NavigationContainer>
  );
}
