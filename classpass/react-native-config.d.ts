declare module 'react-native-config' {
  interface EnvConfig {
    GOOGLE_WEB_CLIENT_ID?: string;
  }
  const Config: EnvConfig;
  export default Config;
}
