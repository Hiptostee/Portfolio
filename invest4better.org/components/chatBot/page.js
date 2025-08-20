import React, { useState, useEffect, useRef } from "react";
import { PaperAirplaneIcon } from "@heroicons/react/24/outline";
import Image from "next/image";
import ReactMarkdown from "react-markdown";
import styles from "../../app/TypingText.module.css";
import remarkGfm from "remark-gfm"; // GitHub Flavored Markdown support
import rehypeReact from "rehype-react"; // For custom HTML rendering
import remarkMath from "remark-math";
import rehypeKatex from "rehype-katex";
import QuestionsModal from "../questionsModal";
import "katex/dist/katex.min.css";
import keys from "../../app/data.json";
import { Doughnut, Bar } from "react-chartjs-2";
import EnvDrawer from "../envDrawer";
import ESGDATA from "../../esgData.json";

import {
  Chart as ChartJS,
  ArcElement,
  Tooltip,
  Legend,
  BarElement,
  CategoryScale,
  LinearScale,
} from "chart.js";

ChartJS.register(
  ArcElement,
  Tooltip,
  Legend,
  BarElement,
  CategoryScale,
  LinearScale
);

import TypingText from "../../components/TypingText";

export default function Chat({ fullText, summary, ticker }) {
  const [messages, setMessages] = useState([]); // Initialize as empty array
  const [input, setInput] = useState("");
  const [isLoading, setIsLoading] = useState(false); // State for loading
  const [isBotTyping, setIsBotTyping] = useState(true); // State for bot typing
  const textareaRef = useRef(null);
  const messagesEndRef = useRef(null);
  const [selected, setSelected] = useState([]);
  const [open, setOpen] = useState(false);
  const [loadingQuestions, setLoadingQuestions] = useState(false);
  const [error, setError] = useState(null); // For error handling
  const [companyInfo, setCompanyInfo] = useState({});
  const [assetAndLiabilities, setAssetAndLiabilities] = useState({});
  const [incomeStatements, setIncomeStatements] = useState([]);
  const [financialRatios, setFinancialRatios] = useState({});
  const [sectorAverageRatios, setSectorAverageRatios] = useState({});
  const [descriptions, setDescriptions] = useState();
  const [currentStockPrice, setCurrentStockPrice] = useState(null);
  const [historicalPrices, setHistoricalPrices] = useState([]);
  const [priceChange, setPriceChange] = useState({});
  const [fiftyTwoWeekHighLow, setFiftyTwoWeekHighLow] = useState({});
  const [movingAverages, setMovingAverages] = useState({});
  const [loading, setLoading] = useState(false);
  const [totalesg, setTotalESG] = useState();
  const [e, setEnvRisk] = useState();
  const [g, setGovRisk] = useState();
  const [s, setSocialRisk] = useState();
  const dataRef = useRef(null);

  const index = Math.floor(Math.random() * keys.length);
  const apiKey = keys[index].api_key;
  function handleSeeData() {
    dataRef.current.scrollIntoView({ behavior: "smooth" });
  }
  const currentYear = new Date().getFullYear();
  useEffect(() => {
    fetchStockInfo();
    getESGScores();
  }, [ticker]);

  const getESGScores = async () => {
    const company = ESGDATA.find((company) => company.Symbol === ticker);
    if (company) {
      setTotalESG(company["Total ESG Risk score"]);
      setEnvRisk(company["Environment Risk Score"]);
      setGovRisk(company["Governance Risk Score"]);
      setSocialRisk(company["Social Risk Score"]);
      const getDescriptions = await fetch("/api/esg", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          ticker: ticker,
          totalESGScore: company["Total ESG Risk score"],
          envScore: company["Environment Risk Score"],
          govScore: company["Governance Risk Score"],
          socialScore: company["Social Risk Score"],
        }),
      });
      const data = await getDescriptions.json();
      setDescriptions(data);
      console.log(data);
    } else {
      console.log("Company not found.");
    }
  };
  const fetchStockInfo = async () => {
    let success = false;
    let attempt = 0;

    while (!success && attempt < keys.length) {
      setError(false);
      try {
        const apiKey = keys[attempt].api_key;

        const [
          profileResponse,
          balanceSheetResponse,
          incomeStatementResponse,
          ratiosResponse,
          stockPriceResponse,
          historicalPricesResponse,
        ] = await Promise.all([
          fetch(
            `https://financialmodelingprep.com/api/v3/profile/${ticker}?apikey=${apiKey}`
          ),
          fetch(
            `https://financialmodelingprep.com/api/v3/balance-sheet-statement/${ticker}?period=annual&apikey=${apiKey}`
          ),
          fetch(
            `https://financialmodelingprep.com/api/v3/income-statement/${ticker}?period=annual&apikey=${apiKey}`
          ),
          fetch(
            `https://financialmodelingprep.com/api/v3/ratios/${ticker}?apikey=${apiKey}`
          ),
          fetch(
            `https://financialmodelingprep.com/api/v3/quote/${ticker}?apikey=${apiKey}`
          ),
          fetch(
            `https://financialmodelingprep.com/api/v3/historical-price-full/${ticker}?timeseries=365&apikey=${apiKey}`
          ),
        ]);

        if (
          profileResponse.ok &&
          balanceSheetResponse.ok &&
          incomeStatementResponse.ok &&
          ratiosResponse.ok &&
          stockPriceResponse.ok &&
          historicalPricesResponse.ok
        ) {
          const profileData = await profileResponse.json();
          const balanceSheetData = await balanceSheetResponse.json();
          const incomeStatementData = await incomeStatementResponse.json();
          const ratiosData = await ratiosResponse.json();
          const stockPriceData = await stockPriceResponse.json();
          const historicalPricesData = await historicalPricesResponse.json();

          // Check if any of the responses contain an empty array or invalid data
          if (
            profileData.length === 0 ||
            balanceSheetData.length === 0 ||
            incomeStatementData.length === 0 ||
            ratiosData.length === 0 ||
            stockPriceData.length === 0 ||
            historicalPricesData.historical.length === 0
          ) {
            setError(true);
            console.error(`No data found for ticker: ${ticker}`);
          } else {
            setCompanyInfo({
              Ticker: profileData[0].symbol,
              "Company Name": profileData[0].companyName,
              Sector: profileData[0].sector,
            });

            setAssetAndLiabilities({
              "Total Assets": balanceSheetData[0].totalAssets / 1_000_000,
              "Total Current Assets":
                balanceSheetData[0].totalCurrentAssets / 1_000_000,
              "Total Non Current Assets":
                balanceSheetData[0].totalNonCurrentAssets / 1_000_000,
              "Total Liabilities":
                balanceSheetData[0].totalLiabilities / 1_000_000,
              "Total Current Liabilities":
                balanceSheetData[0].totalCurrentLiabilities / 1_000_000,
              "Total Non Current Liabilities":
                balanceSheetData[0].totalNonCurrentLiabilities / 1_000_000,
            });

            const simplifiedIncomeStatements = incomeStatementData.map(
              (incomeStatement) => ({
                Revenue: incomeStatement.revenue / 1_000_000,
                "Cost of Revenue": incomeStatement.costOfRevenue / 1_000_000,
                "Gross Profit": incomeStatement.grossProfit / 1_000_000,
                "Operating Expenses":
                  incomeStatement.operatingExpenses / 1_000_000,
                "Net Income": incomeStatement.netIncome / 1_000_000,
              })
            );
            setIncomeStatements(simplifiedIncomeStatements);

            if (ratiosData.length > 0) {
              const mostRecentRatios = ratiosData[0];

              const assetTurnover = mostRecentRatios.assetTurnover || 0;
              const currentRatio = mostRecentRatios.currentRatio || 0;
              const equityMultiplier =
                mostRecentRatios.companyEquityMultiplier || 0;
              const peRatio = mostRecentRatios.priceEarningsRatio || 0;
              const netProfitMargin = mostRecentRatios.netProfitMargin
                ? mostRecentRatios.netProfitMargin * 100
                : 0;

              setFinancialRatios({
                "Asset Turnover": assetTurnover,
                "Current Ratio": currentRatio,
                "Equity Multiplier": equityMultiplier,
                "P/E Ratio": peRatio,
                "Net Profit Margin (in %)": netProfitMargin,
              });
            }

            // Set stock price data
            setCurrentStockPrice(stockPriceData[0].price);
            setPriceChange({
              "1 Day": stockPriceData[0].changesPercentage,
              "1 Week": calculatePriceChange(historicalPricesData, 7),
              "1 Month": calculatePriceChange(historicalPricesData, 30),
            });
            setFiftyTwoWeekHighLow({
              High: stockPriceData[0].yearHigh,
              Low: stockPriceData[0].yearLow,
            });
            setMovingAverages({
              "50-Day": calculateMovingAverage(historicalPricesData, 50),
              "200-Day": calculateMovingAverage(historicalPricesData, 200),
            });

            fetchSectorAverageRatios(profileData[0].sector);
            setLoading(true);

            success = true;
          }
        } else {
          setError(true);
          console.error(`API key failed: ${apiKey}`);
        }
      } catch (error) {
        setError(true);
        console.error(`Error with API key ${keys[attempt].api_key}:`, error);
      }
      attempt++;
    }

    if (!success) {
      console.error("All API keys have been exhausted without success.");
    }
  };

  const fetchSectorAverageRatios = async (sector) => {
    try {
      if (sector in sectorTickers) {
        const ratiosList = [];
        const sectorRatioPromises = sectorTickers[sector].map((sectorTicker) =>
          fetch(
            `https://financialmodelingprep.com/api/v3/ratios/${sectorTicker}?apikey=${apiKey}`
          )
        );
        const sectorRatiosResponses = await Promise.all(sectorRatioPromises);
        const sectorRatiosData = await Promise.all(
          sectorRatiosResponses.map((response) => response.json())
        );

        sectorRatiosData.forEach((ratiosData) => {
          if (ratiosData.length > 0) {
            ratiosList.push({
              "Asset Turnover": ratiosData[0].assetTurnover,
              "Current Ratio": ratiosData[0].currentRatio,
              "Equity Multiplier": ratiosData[0].companyEquityMultiplier,
              "P/E Ratio": ratiosData[0].priceEarningsRatio,
              "Net Profit Margin (in %)": ratiosData[0].netProfitMargin * 100,
            });
          }
        });

        const averageRatios = ratiosList.reduce(
          (acc, ratios) => {
            for (const key in ratios) {
              if (ratios[key] !== "N/A") {
                acc[key] += ratios[key];
              }
            }
            return acc;
          },
          {
            "Asset Turnover": 0,
            "Current Ratio": 0,
            "Equity Multiplier": 0,
            "P/E Ratio": 0,
            "Net Profit Margin (in %)": 0,
          }
        );

        for (const key in averageRatios) {
          averageRatios[key] /= ratiosList.length;
        }

        setSectorAverageRatios(averageRatios);
      } else {
        console.log(`No tickers found for sector: ${sector}`);
      }
    } catch (error) {
      console.error("Error fetching sector average ratios:", error);
    }
  };

  // Helper functions to calculate price change and moving averages
  const calculatePriceChange = (data, days) => {
    if (data.historical.length > days) {
      const previousPrice = data.historical[days].close;
      const currentPrice = data.historical[0].close;
      return ((currentPrice - previousPrice) / previousPrice) * 100;
    }
    return 0;
  };

  const calculateMovingAverage = (data, days) => {
    if (data.historical.length >= days) {
      const prices = data.historical.slice(0, days).map((entry) => entry.close);
      const sum = prices.reduce((a, b) => a + b, 0);
      return sum / days;
    }
    return 0;
  };

  // Donut Chart Data for Assets
  const donutDataAssets = {
    labels: ["Current Assets", "Non-Current Assets"],
    datasets: [
      {
        data: [
          assetAndLiabilities["Total Current Assets"] || 0,
          assetAndLiabilities["Total Non Current Assets"] || 0,
        ],
        backgroundColor: ["#36A2EB", "#FF6384"],
        hoverBackgroundColor: ["#36A2EB", "#FF6384"],
      },
    ],
  };

  // Donut Chart Data for Liabilities
  const donutDataLiabilities = {
    labels: ["Current Liabilities", "Non-Current Liabilities"],
    datasets: [
      {
        data: [
          assetAndLiabilities["Total Current Liabilities"] || 0,
          assetAndLiabilities["Total Non Current Liabilities"] || 0,
        ],
        backgroundColor: ["#FFCE56", "#4BC0C0"],
        hoverBackgroundColor: ["#FFCE56", "#4BC0C0"],
      },
    ],
  };

  // Bar Chart Data comparing user input ticker to sector averages
  const barChartData = {
    labels: Object.keys(financialRatios),
    datasets: [
      {
        label: `${ticker} Ratios`,
        data: Object.values(financialRatios),
        backgroundColor: "#36A2EB",
        hoverBackgroundColor: "#36A2EB",
      },
      {
        label: "Sector Average Ratios",
        data: Object.values(sectorAverageRatios),
        backgroundColor: "#FF6384",
        hoverBackgroundColor: "#FF6384",
      },
    ],
  };

  const [questions, setQuestions] = useState([]);
  const [showEnvData, setShowEnvData] = useState(false);
  useEffect(() => {
    const summaryWithLink = `${summary} **See how this company impacts our environment**: [View environmental data](#)`;

    setMessages([
      {
        sender: "bot",
        text: summaryWithLink,
        isSummary: true,
      },
    ]);
  }, [summary]);
  const components = {
    a: ({ href, children }) => (
      <a
        href={href}
        onClick={(e) => {
          e.preventDefault();
          setOpen(true); /* your logic here */
        }}
      >
        {children}
      </a>
    ),
  };
  // Automatically expand the textarea as the user types
  useEffect(() => {
    if (textareaRef.current) {
      textareaRef.current.style.height = "auto";
      textareaRef.current.style.height = `${textareaRef.current.scrollHeight}px`;
    }
  }, [input]);
  const sectorTickers = {
    Technology: [
      "AAPL",
      "MSFT",
      "GOOGL",
      "AMZN",
      "META",
      "TSLA",
      "NVDA",
      "ADBE",
      "NFLX",
      "CRM",
      "INTC",
      "ORCL",
      "IBM",
      "CSCO",
      "QCOM",
      "TXN",
      "AMD",
      "MU",
      "HPQ",
      "INTU",
    ],
    Healthcare: [
      "JNJ",
      "PFE",
      "MRK",
      "ABT",
      "TMO",
      "BMY",
      "LLY",
      "AMGN",
      "GILD",
      "REGN",
      "VRTX",
      "BIIB",
      "ISRG",
      "ALGN",
      "DHR",
      "SYK",
      "MDT",
      "ZTS",
      "BSX",
      "CI",
    ],
    "Financial Services": [
      "JPM",
      "BAC",
      "WFC",
      "C",
      "GS",
      "MS",
      "AXP",
      "BK",
      "USB",
      "PNC",
      "SCHW",
      "CB",
      "AIG",
      "BLK",
      "TROW",
      "MMC",
      "PRU",
      "CME",
      "SPGI",
      "MCO",
    ],
    "Consumer Discretionary": [
      "HD",
      "MCD",
      "NKE",
      "SBUX",
      "LOW",
      "TJX",
      "AMZN",
      "TSLA",
      "TGT",
      "LULU",
      "EBAY",
      "ROST",
      "GM",
      "F",
      "RCL",
      "NCLH",
      "WYNN",
      "MAR",
      "CMG",
      "BKNG",
    ],
    "Consumer Cyclical": [
      "PG",
      "KO",
      "PEP",
      "PM",
      "WMT",
      "COST",
      "MO",
      "CL",
      "MDLZ",
      "KMB",
      "GIS",
      "HSY",
      "SYY",
      "KHC",
      "STZ",
      "ADM",
      "TAP",
      "CAG",
      "MKC",
      "EL",
    ],
    Energy: [
      "XOM",
      "CVX",
      "COP",
      "PSX",
      "OXY",
      "VLO",
      "MPC",
      "EOG",
      "PXD",
      "KMI",
      "SLB",
      "HAL",
      "BKR",
      "FANG",
      "DVN",
      "APA",
      "HES",
      "CLR",
      "MRO",
      "OKE",
    ],
    Industrials: [
      "HON",
      "MMM",
      "GE",
      "UPS",
      "BA",
      "CAT",
      "LMT",
      "UNP",
      "RTX",
      "DE",
      "FDX",
      "CSX",
      "NOC",
      "GD",
      "ETN",
      "EMR",
      "LHX",
      "ITW",
      "DOV",
      "ROL",
    ],
    "Basic Materials": [
      "LIN",
      "SHW",
      "APD",
      "ECL",
      "PPG",
      "DOW",
      "NUE",
      "FCX",
      "LYB",
      "VMC",
      "MLM",
      "IFF",
      "ALB",
      "AVY",
      "CF",
      "MOS",
      "FMC",
      "EMN",
      "NEM",
      "CC",
    ],
    "Real Estate": [
      "PLD",
      "AMT",
      "CCI",
      "SPG",
      "EQIX",
      "PSA",
      "DLR",
      "AVB",
      "EQR",
      "ESS",
      "O",
      "SBAC",
      "VTR",
      "WELL",
      "PEAK",
      "WY",
      "BXP",
      "ARE",
      "HST",
      "MAA",
    ],
    "Communication Services": [
      "GOOGL",
      "FB",
      "DIS",
      "VZ",
      "T",
      "NFLX",
      "CHTR",
      "CMCSA",
      "TMUS",
      "ATVI",
      "EA",
      "TTWO",
      "FOX",
      "FOXA",
      "VIAC",
      "VIA",
      "OMC",
      "IPG",
      "NWS",
      "NWSA",
    ],
    Utilities: [
      "NEE",
      "DUK",
      "SO",
      "D",
      "AEP",
      "EXC",
      "SRE",
      "XEL",
      "ED",
      "WEC",
      "ES",
      "PEG",
      "AWK",
      "ATO",
      "CMS",
      "EIX",
      "DTE",
      "PNW",
      "NI",
      "CNP",
    ],
  };
  const generateQuestions = async () => {
    setLoadingQuestions(true);
    setError(null); // Reset error state before making request
    setSelected([]); // Clear selected state
    try {
      const response = await fetch("/api/researchQuestions", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({ summary: messages[0].text }),
      });

      if (!response.ok) {
        throw new Error("Network response was not ok.");
      }

      const data = await response.json();
      setQuestions(data.questions);
    } catch (error) {
      setError("Failed to generate questions. Please try again later.");
    } finally {
      setLoadingQuestions(false);
    }
  };

  useEffect(() => {
    setSelected(Array(questions.length).fill(5)); // Initialize with -1 (or null) for each question
  }, []);

  const sendMessage = async () => {
    if (!input.trim()) {
      return; // Prevent sending empty messages
    }
    //get page url

    const newMessages = [...messages, { sender: "user", text: input }];
    setMessages(newMessages);
    setInput("");

    // Add loading message
    const loadingMessage = { sender: "bot", text: "", new: true };
    setMessages((prevMessages) => [...prevMessages, loadingMessage]);
    setIsLoading(true);

    try {
      const botResponse = await followUpResponse();
      setMessages((prevMessages) =>
        prevMessages.map((message, index) =>
          index === prevMessages.length - 1 && message.text === ""
            ? { sender: "bot", text: botResponse, new: true }
            : message
        )
      );
      saveMessage(
        window.location.pathname.split("/").pop(),
        "bot",
        botResponse
      );
    } catch (error) {
      console.error("Error fetching response:", error);
    } finally {
      setIsLoading(false);
    }
  };

  async function followUpResponse() {
    const response = await fetch("/api/followUp", {
      method: "POST",
      body: JSON.stringify({
        fullText: fullText,
        question: input,
        messages: messages,
      }),
    });
    const data = await response.json();
    return data.summary; // Assuming the API returns the bot's response here
  }
  const buttonClassName = `rounded-full mt-2 w-10 h-10 ml-4 text-white ${
    isBotTyping || isLoading
      ? "bg-gray-600 cursor-not-allowed"
      : "bg-green-600 hover:bg-green-500"
  }`;
  const [isOverflowing, setIsOverflowing] = useState(false);
  const containerRef = useRef(null);

  useEffect(() => {
    const container = containerRef.current;
    const checkOverflow = () => {
      if (container) {
        setIsOverflowing(container.scrollHeight > container.clientHeight);
      }
    };

    checkOverflow();
    window.addEventListener("resize", checkOverflow);

    return () => {
      window.removeEventListener("resize", checkOverflow);
    };
  }, []);

  return (
    <>
      <div className="ml-4 w-full relative overflow-x-hidden ">
        <div className="lg:flex w-full h-screen overflow-hidden">
          <div className="relative isolate flex-grow pr-8 sm:px-6 pt-14 lg:pl-0 lg:pr-4">
            <div className="mx-auto max-w-5xl pt-16 flex flex-col h-full">
              <div className="flex flex-col w-full h-[80vh] lg:h-[87vh] rounded-md">
                <div
                  className={`flex mb-40  flex-col-reverse  overflow-hidden p-4 overflow-y-auto overflow-x-auto sm:overflow-x-hidden`}
                >
                  {messages.some((message) => !message.isSummary) && (
                    <div ref={containerRef} className="flex-grow" />
                  )}

                  <div className="max-w-5xl mx-auto space-y-4 w-full">
                    {messages &&
                      messages.map((message, index) => (
                        <div
                          key={index}
                          className={`flex ${
                            message.sender === "bot"
                              ? "justify-start"
                              : "justify-end"
                          }`}
                        >
                          {message.sender === "bot" && (
                            <div className="mr-2 flex-shrink-0">
                              <Image
                                src="/logo.svg"
                                alt="Bot Avatar"
                                width={40}
                                height={40}
                                className="rounded-full"
                              />
                            </div>
                          )}
                          <div
                            className={`${
                              message.sender === "bot"
                                ? "text-gray-900"
                                : "bg-green-700 text-white"
                            } px-3 py-2 rounded-lg max-w-5xl`}
                          >
                            {showEnvData}
                            <div className="flex w-full">
                              <div className="grow max-w-full">
                                {(message.new || message.isSummary) && (
                                  <TypingText
                                    botTyping={setIsBotTyping}
                                    scrollToBottom={messagesEndRef}
                                    markdownText={message}
                                    typingSpeed={1}
                                    setIsBotTyping={setIsBotTyping}
                                    components={components}
                                  />
                                )}

                                {!message.new &&
                                  message.sender === "bot" &&
                                  !message.isSummary && (
                                    <div
                                      className={`${styles.typingContainer} list-decimal`}
                                    >
                                      <ReactMarkdown
                                        remarkPlugins={[remarkGfm, remarkMath]}
                                        rehypePlugins={[rehypeKatex]}
                                        className={styles.typingContainer}
                                      >
                                        {message.text}
                                      </ReactMarkdown>
                                    </div>
                                  )}
                                {!message.new && message.sender === "user" && (
                                  <div>{message.text}</div>
                                )}
                                <div ref={messagesEndRef}></div>
                              </div>
                            </div>
                          </div>
                        </div>
                      ))}
                  </div>
                </div>
                <div
                  aria-hidden="true"
                  className="absolute inset-x-0 top-[calc(100%-30rem)] -z-10 transform-gpu overflow-hidden blur-3xl sm:top-[calc(100%-45rem)]"
                >
                  <div
                    style={{
                      clipPath:
                        "polygon(74.1% 44.1%, 100% 61.6%, 97.5% 26.9%, 85.5% 0.1%, 80.7% 2%, 72.5% 32.5%, 60.2% 62.4%, 52.4% 68.1%, 47.5% 58.3%, 45.2% 34.5%, 27.5% 76.7%, 0.1% 64.9%, 17.9% 100%, 27.6% 76.8%, 76.1% 97.7%, 74.1% 44.1%)",
                    }}
                    className="relative left-[calc(70%+3rem)] aspect-[1155/678] w-[36.125rem] -translate-x-1/3 bg-gradient-to-tr from-green-800 to-green-700 opacity-30 sm:left-[calc(10%+36rem)] sm:w-[72.1875rem]"
                  />
                </div>
              </div>
              <div className="z-50">
                <EnvDrawer
                  open={open}
                  setOpen={setOpen}
                  descriptions={descriptions}
                  totalesg={totalesg}
                  eesg={e}
                  sesg={s}
                  gesg={g}
                  fullText={fullText}
                />
              </div>

              <div className="p-3  xl:w-full  w-[90%] left-0 absolute mr-8 bottom-0 bg-transparent border-t border-gray-700">
                <div className="flex max-w-3xl  mx-auto">
                  <button
                    className="text-green-700 my-auto w-44 text-xs sm:text-base h-10 mr-8 px-4 py-2 border rounded-lg border-green-700"
                    onClick={handleSeeData}
                  >
                    View Stock Data
                  </button>
                  <textarea
                    ref={textareaRef}
                    value={input}
                    onChange={(e) => setInput(e.target.value)}
                    onKeyPress={(e) => {
                      if (e.key === "Enter" && !e.shiftKey) {
                        e.preventDefault();
                        if (!isBotTyping && !isLoading) {
                          sendMessage();
                          setIsBotTyping(true);
                        }
                      }
                    }}
                    rows={1}
                    className="flex-grow bg-transparent overflow-auto border-gray-500 border text-gray-900 rounded-md px-4 py-4 focus:outline-none resize-none "
                    placeholder="Type your message..."
                    style={{
                      lineHeight: "1.5",
                      maxHeight: "100px",
                      scrollbarWidth: "none",
                      msOverflowStyle: "none",
                    }}
                  />
                  <div className="w-10 h-14 ">
                    <button
                      onClick={() => {
                        sendMessage();
                        setIsBotTyping(true);
                      }}
                      disabled={isBotTyping || isLoading}
                      className={buttonClassName}
                    >
                      <PaperAirplaneIcon className="h-6 w-6 mx-auto my-auto" />
                    </button>
                  </div>
                </div>

                <p className=" text-center mb-8  mt-4 text-xs text-green-700">
                  Research Chat can make mistakes. Check important information.
                </p>
              </div>
              {/*             
            {messages.length > 0 && (
              <div aria-hidden="true" className="absolute top-20 right-20 z-10">
                <button
                  disabled={loadingQuestions}
                  onClick={() => generateQuestions()}
                  className={`border px-2 py-1 rounded-full shadow-lg text-xs ${
                    loadingQuestions
                      ? "border-gray-500  cursor-not-allowed" // Styles when loading
                      : "border-green-600 hover:border-green-300 bg-transparent text-white cursor-pointer" // Styles when not loading
                  }`}
                >
                  {loadingQuestions ? "Loading..." : "Generate Questions"}
                </button>
              </div>
            )}
            {error && (
              <div className="absolute top-20 mt-2 right-56 z-10">
                <p className="text-red-500 text-xs">{error}</p>
              </div>
            )}

            <QuestionsModal
              open={open}
              setOpen={setOpen}
              questions={questions}
              selected={selected}
              setSelected={setSelected}
            /> */}
            </div>
          </div>

          <div
            aria-hidden="true"
            className="absolute inset-x-0 top-[calc(100%-30rem)] -z-10 transform-gpu overflow-hidden blur-3xl sm:top-[calc(100%-45rem)]"
          >
            <div
              style={{
                clipPath:
                  "polygon(74.1% 44.1%, 100% 61.6%, 97.5% 26.9%, 85.5% 0.1%, 80.7% 2%, 72.5% 32.5%, 60.2% 62.4%, 52.4% 68.1%, 47.5% 58.3%, 45.2% 34.5%, 27.5% 76.7%, 0.1% 64.9%, 17.9% 100%, 27.6% 76.8%, 76.1% 97.7%, 74.1% 44.1%)",
              }}
              className="relative left-[calc(70%+3rem)] aspect-[1155/678] w-[36.125rem] -translate-x-1/3 bg-gradient-to-tr from-green-800 to-green-700 opacity-30 sm:left-[calc(10%+36rem)] sm:w-[72.1875rem]"
            />
          </div>
        </div>
      </div>
      <div ref={dataRef} className=" mx-auto max-w-7xl px-4">
        <div className="gap-8 items-center">
          <div className="text-center md:text-left">
            <div>
              {/* Display company information */}
              {loading && (
                <>
                  <h2 className="mt-10 text-2xl font-bold text-green-700">
                    Company Information
                  </h2>
                  <div className="bg-white  border-green-700 border rounded-md p-6 mt-4 text-left">
                    {Object.keys(companyInfo).map((key) => (
                      <p key={key} className="text-base my-1 text-green-700">
                        <span className="font-bold text-green-700">{key}:</span>{" "}
                        {companyInfo[key]}
                      </p>
                    ))}
                  </div>

                  {/* Display stock price and historical data */}
                  <h2 className="mt-10 text-2xl font-bold text-green-700">
                    Stock Price and Historical Data
                  </h2>
                  <div className="bg-white  border-green-700 border rounded-md p-6 mt-4 text-left">
                    {currentStockPrice !== null && (
                      <>
                        <p className="text-base my-1 text-green-700">
                          <span className="font-bold text-green-700">
                            Current Price:
                          </span>{" "}
                          {currentStockPrice?.toLocaleString("en-US", {
                            style: "currency",
                            currency: "USD",
                          })}
                        </p>
                        <p className="text-base my-1 text-green-700">
                          <span className="font-bold text-green-700">
                            1 Day Change (%):
                          </span>{" "}
                          {priceChange["1 Day"]?.toFixed(2)}%
                        </p>
                        <p className="text-base my-1 text-green-700">
                          <span className="font-bold text-green-700">
                            1 Week Change (%):
                          </span>{" "}
                          {priceChange["1 Week"]?.toFixed(2)}%
                        </p>
                        <p className="text-base my-1 text-green-700">
                          <span className="font-bold text-green-700">
                            1 Month Change (%):
                          </span>{" "}
                          {priceChange["1 Month"]?.toFixed(2)}%
                        </p>
                        <p className="text-base my-1 text-green-700">
                          <span className="font-bold text-green-700">
                            52-Week High:
                          </span>{" "}
                          {fiftyTwoWeekHighLow.High?.toLocaleString("en-US", {
                            style: "currency",
                            currency: "USD",
                          })}
                        </p>
                        <p className="text-base my-1 text-green-700">
                          <span className="font-bold text-green-700">
                            52-Week Low:
                          </span>{" "}
                          {fiftyTwoWeekHighLow.Low?.toLocaleString("en-US", {
                            style: "currency",
                            currency: "USD",
                          })}
                        </p>
                        <p className="text-base my-1 text-green-700">
                          <span className="font-bold text-green-700">
                            50-Day Moving Average:
                          </span>{" "}
                          {movingAverages["50-Day"]?.toFixed(2)}
                        </p>
                        <p className="text-base my-1 text-green-700">
                          <span className="font-bold text-green-700">
                            200-Day Moving Average:
                          </span>{" "}
                          {movingAverages["200-Day"]?.toFixed(2)}
                        </p>
                      </>
                    )}
                  </div>

                  {/* Display assets and liabilities */}
                  <h2 className="mt-10 text-2xl font-bold text-green-700">
                    Assets and Liabilities (Dollars in millions)
                  </h2>
                  <div className="bg-white  border-green-700 border rounded-md p-6 mt-4 text-left">
                    {Object.keys(assetAndLiabilities).map((key) => (
                      <p key={key} className="text-base my-1 text-green-700">
                        <span className="font-bold text-green-700">{key}:</span>{" "}
                        {assetAndLiabilities[key].toLocaleString("en-US", {
                          style: "currency",
                          currency: "USD",
                        })}
                      </p>
                    ))}
                  </div>

                  {/* Display income statements */}
                  <h2 className="mt-10 text-2xl font-bold text-green-700">
                    Income Statements (Dollars in millions)
                  </h2>
                  <div className="overflow-x-auto">
                    <table className="table-auto w-full mt-4 bg-white  border-green-700 border rounded-md">
                      <thead>
                        <tr className="text-left bg-white text-green-700">
                          <th className="px-4 py-2">{ticker}</th>
                          {incomeStatements.map((_, index) => (
                            <th key={index} className="px-4 py-2">
                              {currentYear +
                                (incomeStatements.length - index - 5)}
                            </th>
                          ))}
                        </tr>
                      </thead>
                      <tbody>
                        {[
                          "Revenue",
                          "Cost of Revenue",
                          "Gross Profit",
                          "Operating Expenses",
                          "Net Income",
                        ].map((metric, metricIndex) => (
                          <tr key={metricIndex} className="text-green-700">
                            <td className="border-t border-gray-500 px-4 py-2 font-bold">
                              {metric}
                            </td>
                            {incomeStatements.map((incomeStatement, index) => (
                              <td
                                key={index}
                                className="border-t border-gray-500 px-4 py-2"
                              >
                                {incomeStatement[metric].toLocaleString(
                                  "en-US",
                                  {
                                    style: "currency",
                                    currency: "USD",
                                  }
                                )}
                              </td>
                            ))}
                          </tr>
                        ))}
                      </tbody>
                    </table>
                  </div>

                  {/* Display financial ratios */}
                  <h2 className="mt-10 text-2xl font-bold text-green-700">
                    Financial Ratios
                  </h2>
                  <div className="bg-white  border-green-700 border rounded-md p-6 mt-4 text-left">
                    {Object.keys(financialRatios).map((key) => (
                      <p key={key} className="text-base my-1 text-green-700">
                        <span className="font-bold text-green-700">{key}:</span>{" "}
                        {financialRatios[key].toFixed(2)}
                      </p>
                    ))}
                  </div>

                  {/* Display sector average ratios */}
                  <h2 className="mt-10 text-2xl font-bold text-green-700">
                    Average Financial Ratios for Sector
                  </h2>
                  <div className="bg-white  border-green-700 border rounded-md p-6 mt-4 text-left">
                    {Object.keys(sectorAverageRatios).map((key) => (
                      <p key={key} className="text-base my-1 text-green-700">
                        <span className="font-bold text-green-700">{key}:</span>{" "}
                        {sectorAverageRatios[key].toFixed(2)}
                      </p>
                    ))}
                  </div>

                  {/* Donut Chart for Current vs. Non-Current Assets */}
                  <div className="flex flex-col md:flex-row justify-center items-center mx-auto w-full mt-10">
                    <div className="flex flex-col items-center text-center mx-4">
                      <h2 className="text-2xl font-bold text-green-700">
                        Asset Breakdown
                      </h2>
                      <div className="w-64 h-64 mt-4">
                        <Doughnut
                          data={donutDataAssets}
                          options={{
                            plugins: {
                              legend: {
                                display: false,
                              },
                            },
                          }}
                        />
                      </div>
                    </div>
                    <div className="flex flex-col items-center text-center mx-4 mt-10 md:mt-0">
                      <h2 className="text-2xl font-bold text-green-700">
                        Liability Breakdown
                      </h2>
                      <div className="w-64 h-64 mt-4">
                        <Doughnut
                          data={donutDataLiabilities}
                          options={{
                            plugins: {
                              legend: {
                                display: false,
                              },
                            },
                          }}
                        />
                      </div>
                    </div>
                  </div>
                </>
              )}

              {/* Bar Chart comparing ticker ratios to sector averages */}
            </div>
            {loading && (
              <>
                <h2 className="mt-10 text-2xl font-bold text-green-700 text-center">
                  Comparison of Financial Ratios: {ticker} vs. Sector Averages
                </h2>
                <div className="overflow-x-scroll overflow-y-hidden">
                  <div className="w-[800px] h-96  border-green-300 border-2 rounded-xl bg-[#003927] text-green-700 mx-auto mt-6">
                    <Bar
                      data={barChartData}
                      options={{
                        plugins: {
                          legend: {
                            display: true,
                            position: "top",
                            labels: {
                              color: "white", // Make legend text white
                              font: {
                                size: 12, // Font size of the legend
                                weight: "bold", // Bold legend text
                              },
                            },
                          },
                        },
                        scales: {
                          x: {
                            beginAtZero: true,
                            ticks: {
                              color: "white", // Make x-axis text white
                              font: {
                                size: 10, // Font size of x-axis labels
                                weight: "bold", // Bold x-axis labels
                              },
                            },
                            grid: {
                              color: "rgba(255, 255, 255, 0.2)", // Light grid lines for contrast
                            },
                          },
                          y: {
                            beginAtZero: true,
                            ticks: {
                              color: "white", // Make y-axis text white
                              font: {
                                size: 10, // Font size of y-axis labels
                                weight: "bold", // Bold y-axis labels
                              },
                            },
                            grid: {
                              color: "rgba(255, 255, 255, 0.2)", // Light grid lines for contrast
                            },
                          },
                        },
                      }}
                    />
                  </div>
                </div>
              </>
            )}
          </div>
        </div>
      </div>
    </>
  );
}
