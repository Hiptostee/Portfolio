import { Inter } from "next/font/google";
import "./globals.css";
import Script from "next/script";
const inter = Inter({ subsets: ["latin"] });
import "katex/dist/katex.min.css";

export const metadata = {
  title: "Invest4Better",
  description:
    "Enhance your research efficiency with Invest4Better's AI-powered summarizing tool. Our cutting-edge technology quickly and accurately condenses complex academic papers into clear, concise summaries, saving you time and effort.",
  icons: {
    icon: "/favicon.ico",
  },
  openGraph: {
    title: "Invest4Better",
    url: "https://environmentstocks.vercel.app/",
    description:
      "Enhance your research efficiency with Invest4Better's AI-powered summarizing tool. Our cutting-edge technology quickly and accurately condenses complex academic papers into clear, concise summaries, saving you time and effort.",
    siteName: "Invest4Better",
  },
  twitter: {
    title: "Invest4Better",
    url: "https://environmentstocks.vercel.app/",
    description:
      "Enhance your research efficiency with Invest4Better's AI-powered summarizing tool. Our cutting-edge technology quickly and accurately condenses complex academic papers into clear, concise summaries, saving you time and effort.",
    siteName: "Invest4Better",
  },
  keywords: [
    "research",
    "research chat",
    "finance",
    "finance chat",
    "10k reports",
    "research assistant",
    "financial analysis",
    "investment strategies",
    "chatbot technology",
    "10-K analysis",
    "data science",
    "AI research",
    "machine learning",
    "natural language processing",
    "financial reports",
    "corporate filings",
    "automated summarization",
    "investment insights",
    "research tools",
    "financial modeling",
    "chatbot development",
    "industry research",
    "quantitative analysis",
    "predictive analytics",
    "research automation",
    "economic forecasting",
    "business intelligence",
    "data visualization",
    "market trends",
    "portfolio management",
    "sentiment analysis",
    "financial forecasting",
    "data mining",
    "research methodologies",
    "risk assessment",
    "algorithmic trading",
    "financial technology",
    "chatbot interactions",
    "science data",
    "research chatbots",
    "10-K disclosures",
    "financial summaries",
    "quantitative research",
    "science communication",
    "automated data extraction",
    "corporate research",
    "chatbot algorithms",
    "financial insights",
    "business research",
    "research questions",
    "predictive modeling",
    "data analysis tools",
    "investment research",
    "research automation tools",
    "chatbot responses",
  ],
};
export default function RootLayout({ children }) {
  return (
    <html lang="en">
      <head>
        <meta charSet="UTF-8" />

        {/* other head elements */}
      </head>
      <Script
        async
        src="https://www.googletagmanager.com/gtag/js?id=G-L58FD1LW2G"
      ></Script>
      <Script async id="google-analytics" strategy="afterInteractive">
        {` 
  window.dataLayer = window.dataLayer || [];
  function gtag(){dataLayer.push(arguments);}
  gtag('js', new Date());
  gtag('config', 'G-L58FD1LW2G');
  `}
      </Script>

      <body className={inter.className}>{children}</body>
    </html>
  );
}
