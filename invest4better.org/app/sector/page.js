"use client";

import React, { useState, useMemo, useEffect, useRef } from "react";
import { Dialog, DialogPanel } from "@headlessui/react";
import ESGBarChart from "../../components/esgbarchart"; // Custom chart component
import { Bars3Icon, XMarkIcon, CheckIcon } from "@heroicons/react/24/outline";
import Image from "next/image";
import Link from "next/link";
import keys from "../../app/data.json"; // Assuming this file contains API keys
import techCompanies from "../../esgData.json"; // Tech companies data
import { useSearchParams } from "next/navigation";
import ReactMarkdown from "react-markdown";
import remarkGfm from "remark-gfm";
import styles from "../TypingText.module.css";
import Info from "../../components/indexinfo";

const navigation = [
  { name: "Back", href: "/sectors" },
  { name: "Home", href: "/" },
];

export default function ChatPage() {
  const [mobileMenuOpen, setMobileMenuOpen] = useState(false);
  const [selectedCompanies, setSelectedCompanies] = useState([]);
  const [companyInfo, setCompanyInfo] = useState({});
  const [assetAndLiabilities, setAssetAndLiabilities] = useState({});
  const [incomeStatements, setIncomeStatements] = useState({});
  const [financialRatios, setFinancialRatios] = useState({});
  const [currentStockPrice, setCurrentStockPrice] = useState({});
  const [priceChange, setPriceChange] = useState({});
  const [fiftyTwoWeekHighLow, setFiftyTwoWeekHighLow] = useState({});
  const [movingAverages, setMovingAverages] = useState({});
  const [infoOpen, setInfoOpen] = useState(false);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(false);
  const [recommendation, setRecommendation] = useState(""); // GPT recommendation state
  const [loadingGPT, setLoadingGPT] = useState(false); // Loading state for GPT
  const search = useSearchParams().get("sector");

  const recommendationRef = useRef(null); // Reference to the recommendation div

  const techSectorData = useMemo(
    () =>
      Array.isArray(techCompanies)
        ? techCompanies
            .filter((company) => company.Sector === search)
            .map((company) => ({
              name: company.Name,
              ticker: company.Symbol,
              e: company["Environment Risk Score"],
              s: company["Social Risk Score"],
              g: company["Governance Risk Score"],
              esg: company["Total ESG Risk score"],
            }))
        : [],
    [search]
  );

  const esgData = useMemo(
    () =>
      selectedCompanies.map((company) => ({
        e: company.e,
        s: company.s,
        g: company.g,
      })),
    [selectedCompanies]
  );

  const tickers = useMemo(
    () => selectedCompanies.map((company) => company.ticker),
    [selectedCompanies]
  );

  const calculatePriceChange = (data, days) => {
    if (data.historical && data.historical.length > days) {
      const previousPrice = data.historical[days].close;
      const currentPrice = data.historical[0].close;
      return ((currentPrice - previousPrice) / previousPrice) * 100;
    }
    return 0;
  };

  const calculateMovingAverage = (data, days) => {
    if (data.historical && data.historical.length >= days) {
      const prices = data.historical.slice(0, days).map((entry) => entry.close);
      const sum = prices.reduce((a, b) => a + b, 0);
      return sum / days;
    }
    return 0;
  };

  const calculateESGScore = (company) => {
    const { e, s, g } = company;
    return e + s + g; // Lower is better
  };

  const calculateIndex = (company) => {
    const esgScore = calculateESGScore(company);
    const ratios = financialRatios[company.ticker];

    if (ratios && esgScore) {
      const { "Net Profit Margin (in %)": margin } = ratios;
      return margin / esgScore; // Adjust the formula as needed
    }
    return 0; // Default index value if no data
  };

  const generateRecommendationPrompt = () => {
    const esgScale = `
**ESG Score Interpretation:**
- **0-10:** Negligible
- **10-20:** Low
- **20-40:** High
- **Above 40:** Severe

*Note: In this context, lower ESG scores are better.*
    `;

    const companyDataStrings = selectedCompanies
      .map((company) => {
        const eScore = company.e || "-";
        const sScore = company.s || "-";
        const gScore = company.g || "-";
        const totalESGScore = company.esg || "-";

        const financialData = companyInfo[company.ticker]
          ? `
- Revenue Growth: ${companyInfo[company.ticker].revenueGrowth || "-"}%
- Profit Margin: ${
              financialRatios[company.ticker]["Net Profit Margin (in %)"] || "-"
            }%
- P/E Ratio: ${financialRatios[company.ticker]["P/E Ratio"] || "-"}
- Dividend Yield: ${financialRatios[company.ticker]["Dividend Yield"] || "-"}%
          `
          : `
- Revenue Growth: -
- Profit Margin: -
- P/E Ratio: -
- Dividend Yield: -
          `;

        return `
${company.name} (${company.ticker}):
- Environmental Score: ${eScore}
- Social Score: ${sScore}
- Governance Score: ${gScore}
- Total ESG Score: ${totalESGScore}
- I4B Index: ${calculateIndex(company)}
${financialData}
        `;
      })
      .join("\n");

    return `
You are a financial analyst specializing in ESG investing. Based on the following data for the selected companies, 
analyze and recommend which stock to consider buying. Make sure to actually give 1 out of the options of stocks on 
which stock to invest in, based on all the scores and information provided. The index is our own, it is the net 
profit margin divided by the esg score. The higher the better. Do not rewrite all of the financial metrics, 
just relevant ones. Use markdown, make sure to spit out the entire response, leave no words out..

${esgScale}

${companyDataStrings}

**Guidelines:**
- **Equal Consideration:** Ensure that both financial performance and ESG factors are equally evaluated.
- **ESG Interpretation:** Remember that lower ESG scores are better based on the provided scale.
- **Comprehensive Analysis:** Avoid relying solely on ESG scores; consider financial metrics such as revenue growth, 
   profit margin, P/E ratio, and dividend yield.
- **Avoid Greenwashing:** Ensure that low ESG scores reflect genuine sustainable practices rather than superficial 'greenwashing.'
- **Render as markdown
    `;
  };

  const fetchGPTRecommendation = async () => {
    setLoadingGPT(true);
    try {
      const prompt = generateRecommendationPrompt();
      const response = await fetch(
        "https://api.openai.com/v1/chat/completions",
        {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
            Authorization: `Bearer ${process.env.OPENAI_API_KEY}`,
          },
          body: JSON.stringify({
            model: "gpt-4o-mini",
            messages: [
              {
                role: "system",
                content:
                  "You are a financial analyst specializing in ESG investing.",
              },
              { role: "user", content: prompt },
            ],
            max_tokens: 1500,
          }),
        }
      );

      const data = await response.json();
      if (data.choices && data.choices.length > 0) {
        console.log(data.choices[0].message.content);
        setRecommendation(data.choices[0].message.content);
        // Scroll down to the recommendation
        if (recommendationRef.current) {
          recommendationRef.current.scrollIntoView({ behavior: "smooth" });
        }
      } else {
        setRecommendation("No recommendation available.");
      }
    } catch (error) {
      console.error("Error fetching GPT recommendation: ", error);
      setRecommendation(
        "An error occurred while generating the recommendation."
      );
    } finally {
      setLoadingGPT(false);
    }
  };

  const fetchStockInfo = async (ticker) => {
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

          if (
            profileData.length === 0 ||
            balanceSheetData.length === 0 ||
            incomeStatementData.length === 0 ||
            ratiosData.length === 0 ||
            stockPriceData.length === 0 ||
            !historicalPricesData.historical
          ) {
            setError(true);
            console.error(`No data found for ticker: ${ticker}`);
          } else {
            setCompanyInfo((prev) => ({
              ...prev,
              [ticker]: {
                companyName: profileData[0].companyName,
                revenueGrowth: incomeStatementData[0].revenueGrowth || "-",
              },
            }));

            setAssetAndLiabilities((prev) => ({
              ...prev,
              [ticker]: {
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
              },
            }));
            setIncomeStatements((prev) => ({
              ...prev,
              [ticker]: incomeStatementData.map((incomeStatement) => ({
                "Total Revenue": incomeStatement.revenue / 1_000_000,
                "Cost of Revenue": incomeStatement.costOfRevenue / 1_000_000,
                "Gross Profit": incomeStatement.grossProfit / 1_000_000,
                "Operating Expenses":
                  incomeStatement.operatingExpenses / 1_000_000,
                "Net Income": incomeStatement.netIncome / 1_000_000,
              })),
            }));

            if (ratiosData.length > 0) {
              const mostRecentRatios = ratiosData[0];

              setFinancialRatios((prev) => ({
                ...prev,
                [ticker]: {
                  "Asset Turnover": mostRecentRatios.assetTurnover || 0,
                  "Current Ratio": mostRecentRatios.currentRatio || 0,
                  "Equity Multiplier":
                    mostRecentRatios.companyEquityMultiplier || 0,
                  "P/E Ratio": mostRecentRatios.priceEarningsRatio || 0,
                  "Net Profit Margin (in %)": mostRecentRatios.netProfitMargin
                    ? mostRecentRatios.netProfitMargin * 100
                    : 0,
                  "Dividend Yield": mostRecentRatios.dividendYield || 0,
                },
              }));
            }

            setCurrentStockPrice((prev) => ({
              ...prev,
              [ticker]: stockPriceData[0].price,
            }));

            setPriceChange((prev) => ({
              ...prev,
              [ticker]: {
                "1 Day": stockPriceData[0].changesPercentage,
                "1 Week": calculatePriceChange(historicalPricesData, 7),
                "1 Month": calculatePriceChange(historicalPricesData, 30),
              },
            }));
            setFiftyTwoWeekHighLow((prev) => ({
              ...prev,
              [ticker]: {
                High: stockPriceData[0].yearHigh,
                Low: stockPriceData[0].yearLow,
              },
            }));
            setMovingAverages((prev) => ({
              ...prev,
              [ticker]: {
                "50-Day": calculateMovingAverage(historicalPricesData, 50),
                "200-Day": calculateMovingAverage(historicalPricesData, 200),
              },
            }));
          }
          success = true; // Set success to true when data is fetched
        } else {
          throw new Error("One or more responses were not OK");
        }
      } catch (err) {
        console.error("Error fetching stock info: ", err);
        setError(true);
        attempt++;
      }
    }
  };

  const handleCompanyClick = (company) => {
    setRecommendation("");
    const isSelected = selectedCompanies.some(
      (c) => c.ticker === company.ticker
    );

    if (isSelected) {
      // Deselect the company
      setSelectedCompanies((prev) =>
        prev.filter((c) => c.ticker !== company.ticker)
      );
      // Remove related financial data
      setIncomeStatements((prev) => {
        const { [company.ticker]: _, ...rest } = prev;
        return rest;
      });
      setFinancialRatios((prev) => {
        const { [company.ticker]: _, ...rest } = prev;
        return rest;
      });
      setCurrentStockPrice((prev) => {
        const { [company.ticker]: _, ...rest } = prev;
        return rest;
      });
      setPriceChange((prev) => {
        const { [company.ticker]: _, ...rest } = prev;
        return rest;
      });
      setFiftyTwoWeekHighLow((prev) => {
        const { [company.ticker]: _, ...rest } = prev;
        return rest;
      });
      setMovingAverages((prev) => {
        const { [company.ticker]: _, ...rest } = prev;
        return rest;
      });
      setCompanyInfo((prev) => {
        const { [company.ticker]: _, ...rest } = prev;
        return rest;
      });
    } else if (selectedCompanies.length < 3) {
      // Select the company
      setSelectedCompanies((prev) => [...prev, company]);
      fetchStockInfo(company.ticker);
    } else {
      // Notify user they've reached the selection limit
      alert("You can only select up to 3 companies.");
    }
  };

  return (
    <>
      <header className="absolute inset-x-0 top-0 z-10">
        <nav
          aria-label="Global"
          className="mx-auto flex  items-center justify-between p-6 lg:px-8"
        >
          <div className="flex lg:flex-1">
            <Link href="/">
              <Image
                alt="Your Company"
                src="/logo.svg"
                width={200}
                height={200}
                className="h-16 rounded-full w-auto"
              />
            </Link>
          </div>
          <div className="flex lg:hidden">
            <button
              type="button"
              onClick={() => setMobileMenuOpen(true)}
              className="-m-2.5 inline-flex items-center justify-center rounded-md p-2.5 text-green-600"
            >
              <span className="sr-only">Open main menu</span>
              <Bars3Icon aria-hidden="true" className="h-6 w-6" />
            </button>
          </div>
          <div className="hidden lg:flex lg:gap-x-12">
            {navigation.map((item) => (
              <Link
                key={item.name}
                href={item.href}
                className="text-sm font-semibold leading-6 text-green-600 hover:underline rounded-lg px-3 py-2"
              >
                {item.name}
              </Link>
            ))}
          </div>
        </nav>
        <Dialog
          open={mobileMenuOpen}
          onClose={setMobileMenuOpen}
          className="lg:hidden"
        >
          <div className="fixed inset-0 z-10" />
          <DialogPanel className="fixed inset-y-0 right-0 z-10 w-full overflow-y-auto bg-gray-900 px-6 py-6 sm:max-w-sm sm:ring-1 sm:ring-gray-900/10">
            <div className="flex items-center justify-between">
              <Link href="/">
                <Image
                  alt="Your Company"
                  src="/logo.svg"
                  width={200}
                  height={200}
                  className="h-16 rounded-full w-auto"
                />
              </Link>
              <button
                type="button"
                onClick={() => setMobileMenuOpen(false)}
                className="-m-2.5 rounded-md p-2.5 text-gray-200"
              >
                <span className="sr-only">Close menu</span>
                <XMarkIcon aria-hidden="true" className="h-6 w-6" />
              </button>
            </div>
            <div className="mt-6 flow-root">
              <div className="-my-6 divide-y divide-gray-500/10">
                <div className="space-y-2 py-6">
                  {navigation.map((item) => (
                    <Link
                      key={item.name}
                      href={item.href}
                      className="-mx-3 block rounded-lg px-3 py-2 text-base font-semibold leading-7 text-green-600 hover:underline"
                    >
                      {item.name}
                    </Link>
                  ))}
                </div>
              </div>
            </div>
          </DialogPanel>
        </Dialog>
      </header>
      <Info open={infoOpen} setOpen={setInfoOpen} />

      <main className="flex bg-gray-900 h-max flex-col">
        <div className="flex-grow mt-20 flex flex-col p-6">
          <p className="text-base font-bold mb-6 text-red-600">
            THIS IS NOT FINANCIAL ADVICE
          </p>
          <h1 className="text-2xl font-bold mb-6 text-green-600">
            Select Up to 3 {search} Companies to Compare
          </h1>

          <div className="flex flex-row gap-6 isolate relative">
            <div
              aria-hidden="true"
              className="absolute inset-x-0 -top-40 -z-10 transform-gpu overflow-hidden blur-3xl sm:-top-80"
            >
              <div
                style={{
                  clipPath:
                    "polygon(74.1% 44.1%, 100% 61.6%, 97.5% 26.9%, 85.5% 0.1%, 80.7% 2%, 72.5% 32.5%, 60.2% 62.4%, 52.4% 68.1%, 47.5% 58.3%, 45.2% 34.5%, 27.5% 76.7%, 0.1% 64.9%, 17.9% 100%, 27.6% 76.8%, 76.1% 97.7%, 74.1% 44.1%)",
                }}
                className="relative left-[calc(50%-11rem)] aspect-[1155/678] w-[36.125rem] -translate-x-1/2 rotate-[30deg] bg-gradient-to-tr from-emerald-900 to-emerald-700 opacity-30 sm:left-[calc(50%-30rem)] sm:w-[72.1875rem]"
              />
            </div>
            {/* Left Column for Companies */}
            <div className="flex flex-col md:flex-row gap-4">
              {/* Left Column (Company Grid) */}
              <div className="w-full md:w-1/2">
                <div className="grid grid-cols-2 sm:grid-cols-3 gap-4">
                  {techSectorData.map((company) => (
                    <button
                      key={company.ticker}
                      className={`relative p-4 text-gray-400 border rounded-lg shadow-md transition duration-200 ease-in-out ${
                        selectedCompanies.some(
                          (selected) => selected.ticker === company.ticker
                        )
                          ? "bg-gray-700"
                          : "bg-gray-800"
                      } cursor-pointer hover:bg-gray-700`}
                      onClick={() => handleCompanyClick(company)}
                      aria-pressed={selectedCompanies.some(
                        (selected) => selected.ticker === company.ticker
                      )}
                    >
                      {selectedCompanies.some(
                        (selected) => selected.ticker === company.ticker
                      ) && (
                        <CheckIcon className="absolute top-2 right-2 h-5 w-5 text-green-600" />
                      )}
                      <h2 className="font-semibold">
                        {company.name} ({company.ticker})
                      </h2>
                      <p>
                        ESG Score: {company.esg ? company.esg : "Not Found"}
                      </p>
                    </button>
                  ))}
                </div>
              </div>

              {/* Right Column for ESG Chart and Financial Data */}
              <div className="w-full md:w-1/2 flex flex-col">
                {/* Button Above the Graph */}
                {!recommendation && (
                  <div className="flex items-center justify-center mb-4">
                    <button
                      onClick={fetchGPTRecommendation}
                      className="px-6 py-3 bg-green-700 text-white rounded-md hover:bg-green-800"
                      disabled={selectedCompanies.length === 0 || loadingGPT}
                    >
                      {loadingGPT ? "Generating..." : "Get recommendation"}
                    </button>
                  </div>
                )}
                {recommendation && (
                  <div
                    ref={recommendationRef}
                    className={`${styles.typingContainer} list-decimal text-gray-200`}
                  >
                    <h3 className="text-lg font-semibold mb-2">
                      Recommendation
                    </h3>
                    <ReactMarkdown
                      children={recommendation}
                      styles={styles.typingContainer}
                      remarkPlugins={[remarkGfm]}
                    />
                  </div>
                )}
                {/* ESG Chart */}
                <div className="flex flex-col items-start">
                  <div className="flex items-center mb-2">
                    <h2 className="text-xl font-bold text-green-600">
                      ESG Scores
                    </h2>
                  </div>
                  <div className="w-full">
                    <ESGBarChart data={esgData} ticker={tickers} />
                  </div>
                </div>

                {/* Financial Data Table */}
                {selectedCompanies.length > 0 && (
                  <div className="mt-6 overflow-auto text-gray-300 w-full">
                    <div className="flex items-center mb-2">
                      <h2 className="text-xl font-bold text-green-600">
                        Company Financial Data
                      </h2>
                    </div>
                    <div className="w-full">
                      <table
                        style={{
                          width: "100%",
                          borderCollapse: "collapse",
                          marginTop: "1rem",
                          marginBottom: "1rem",
                          backgroundColor: "#006828",
                          border: "1px solid #006828",
                          borderRadius: "8px",
                          overflow: "hidden",
                          boxShadow: "0 4px 8px rgba(0, 0, 0, 0.1)",
                        }}
                      >
                        <thead>
                          <tr
                            style={{
                              backgroundColor: "#006828",
                              color: "#ffffff",
                            }}
                          >
                            <th
                              style={{
                                fontWeight: 700,
                                backgroundColor: "#006828",
                                color: "#ffffff",
                                padding: "1rem",
                                textAlign: "left",
                                borderBottom: "1px solid #eaeaea",
                                borderLeft: "1px solid #eaeaea",
                                borderRight: "1px solid #eaeaea",
                              }}
                            >
                              Metric
                            </th>
                            {selectedCompanies.map((company) => (
                              <th
                                key={company.ticker} // Add a key prop here
                                style={{
                                  fontWeight: 700,
                                  backgroundColor: "#006828",
                                  color: "#ffffff",
                                  padding: "1rem",
                                  textAlign: "left",
                                  borderBottom: "1px solid #eaeaea",
                                  borderLeft: "none",
                                  borderRight: "none",
                                }}
                              >
                                {company.ticker}
                              </th>
                            ))}
                          </tr>
                        </thead>
                        <tbody>
                          {[
                            {
                              label: "Company",
                              value: (company) =>
                                companyInfo[company.ticker]?.["companyName"] ||
                                company.name,
                            },
                            {
                              label: "Total Revenue",
                              value: (company) =>
                                incomeStatements[company.ticker]?.[0]?.[
                                  "Total Revenue"
                                ]?.toFixed(2) || "-",
                            },
                            {
                              label: "Net Income (in millions)",
                              value: (company) =>
                                incomeStatements[company.ticker]?.[0]?.[
                                  "Net Income"
                                ]?.toFixed(2) || "-",
                            },
                            {
                              label: "Net Profit Margin (%)",
                              value: (company) =>
                                financialRatios[company.ticker]?.[
                                  "Net Profit Margin (in %)"
                                ]?.toFixed(2) || "-",
                            },
                            {
                              label: "Current Ratio",
                              value: (company) =>
                                financialRatios[company.ticker]?.[
                                  "Current Ratio"
                                ]?.toFixed(2) || "-",
                            },
                            {
                              label: "Current Price",
                              value: (company) =>
                                currentStockPrice[company.ticker] || "-",
                            },
                            {
                              label: "P/E Ratio",
                              value: (company) =>
                                financialRatios[company.ticker]?.[
                                  "P/E Ratio"
                                ]?.toFixed(2) || "-",
                            },
                            {
                              label: "Dividend Yield (%)",
                              value: (company) =>
                                financialRatios[company.ticker]?.[
                                  "Dividend Yield"
                                ]?.toFixed(2) || "-",
                            },
                            {
                              label: (
                                <span
                                  onClick={() => setInfoOpen(true)}
                                  className="cursor-pointer text-green-500 hover:text-green-600 hover:underline"
                                >
                                  I4B INDEX
                                  <sup className="text-sm text-purple-600">
                                    ⓘ
                                  </sup>
                                </span>
                              ),
                              value: (company) =>
                                calculateIndex(company).toFixed(2),
                            },
                          ].map(({ label, value }) => (
                            <tr
                              key={label}
                              style={{
                                backgroundColor: "#1f2937",
                                cursor: "pointer",
                              }}
                            >
                              <td
                                style={{
                                  padding: "0.75rem",
                                  borderBottom: "1px solid #eaeaea",
                                  borderLeft: "1px solid #eaeaea",
                                }}
                              >
                                {label}
                              </td>
                              {selectedCompanies.map((company) => (
                                <td
                                  key={company.ticker} // Add a key prop here
                                  style={{
                                    padding: "0.75rem",
                                    borderBottom: "1px solid #eaeaea",
                                    borderLeft: "none",
                                    borderRight: "none",
                                    backgroundColor: "#1f2937",
                                  }}
                                >
                                  {value(company)}
                                </td>
                              ))}
                            </tr>
                          ))}
                        </tbody>
                      </table>

                      <p className="text-sm text-gray-500 mt-2">
                        I4B Index Ratings
                      </p>
                      <p className="text-sm text-gray-500">
                        {"<"} 1: Poor | 1-2: Average | 2-3: Good | 3-4:
                        Excellent
                      </p>
                    </div>
                  </div>
                )}

                {/* Recommendation Output */}

                {/* Error Handling */}
                {error && (
                  <div className="mt-4 text-red-600">
                    <p>
                      There was an error fetching the data. Please try again
                      later.
                    </p>
                    <button
                      onClick={() => {
                        if (selectedCompanies.length > 0) {
                          fetchStockInfo(
                            selectedCompanies[selectedCompanies.length - 1]
                              .ticker
                          );
                        }
                      }}
                      className="mt-2 px-4 py-2 bg-red-500 text-white rounded"
                    >
                      Retry
                    </button>
                  </div>
                )}
                <div
                  aria-hidden="true"
                  className="absolute inset-x-0 top-[calc(100%-30rem)] -z-10 transform-gpu overflow-hidden blur-3xl sm:top-[calc(100%-45rem)]"
                >
                  <div
                    style={{
                      clipPath:
                        "polygon(74.1% 44.1%, 100% 61.6%, 97.5% 26.9%, 85.5% 0.1%, 80.7% 2%, 72.5% 32.5%, 60.2% 62.4%, 52.4% 68.1%, 47.5% 58.3%, 45.2% 34.5%, 27.5% 76.7%, 0.1% 64.9%, 17.9% 100%, 27.6% 76.8%, 76.1% 97.7%, 74.1% 44.1%)",
                    }}
                    className="relative left-[calc(50%-11rem)] aspect-[1155/678] w-[36.125rem] -translate-x-1/2 rotate-[30deg] bg-gradient-to-tr from-emerald-900 to-emerald-700 opacity-30 sm:left-[calc(50%-30rem)] sm:w-[72.1875rem]"
                  />
                </div>
              </div>
            </div>
          </div>
        </div>
      </main>
    </>
  );
}
