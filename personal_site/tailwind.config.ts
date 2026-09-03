import type { Config } from "tailwindcss";

const config: Config = {
  content: ["./app/**/*.{js,ts,jsx,tsx}"],
  theme: {
    extend: {
      colors: {
        ink: "#07152c",
        navy: "#0b2140",
        cyan: "#80f4ed",
        amber: "#ffb45e",
        coral: "#ff7159",
        violet: "#8b7cff",
      },
      fontFamily: {
        sans: ["Arial", "Helvetica", "sans-serif"],
      },
      keyframes: {
        rise: { "0%": { opacity: "0", transform: "translateY(24px)" }, "100%": { opacity: "1", transform: "translateY(0)" } },
        drift: { "0%, 100%": { transform: "translate3d(0, 0, 0)" }, "50%": { transform: "translate3d(0, -14px, 0)" } },
        pulseSoft: { "0%, 100%": { opacity: "0.35" }, "50%": { opacity: "0.8" } },
        scan: { "0%": { transform: "translateY(-110%)" }, "100%": { transform: "translateY(700%)" } },
      },
      animation: {
        rise: "rise 800ms cubic-bezier(.16,1,.3,1) both",
        drift: "drift 7s ease-in-out infinite",
        pulseSoft: "pulseSoft 4s ease-in-out infinite",
        scan: "scan 5s linear infinite",
      },
    },
  },
  plugins: [],
};

export default config;
