// /app/chat/page.jsx
"use client";

import { useState, useMemo } from "react";
import { useSearchParams } from "next/navigation";
import techCompanies from "../../esgData.json";

// Custom Hooks
import { useStockData } from "../../hooks/useStockData";
import { useGptRecommendation } from "../../hooks/GPTRecommendation";

// Components
import Header from "../../components/header";
import CompanySelector from "../../components/companySelector";
import Dashboard from "../../components/dashboard";
import Info from "../../components/indexinfo";

export default function ChatPage() {
  const [selectedCompanies, setSelectedCompanies] = useState([]);
  const [infoOpen, setInfoOpen] = useState(false);
  const search = useSearchParams().get("sector");

  const { financialData, loading, error, fetchStockInfo, removeStockInfo } =
    useStockData();
  const {
    recommendation,
    loadingGPT,
    fetchGPTRecommendation,
    generateRecommendationPrompt,
    recommendationRef,
    setRecommendation,
  } = useGptRecommendation();

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
    () => selectedCompanies.map((c) => ({ e: c.e, s: c.s, g: c.g })),
    [selectedCompanies]
  );

  const tickers = useMemo(
    () => selectedCompanies.map((c) => c.ticker),
    [selectedCompanies]
  );

  const handleCompanyClick = (company) => {
    // Clear previous recommendation when selection changes
    if (recommendation) setRecommendation("");

    const isSelected = selectedCompanies.some(
      (c) => c.ticker === company.ticker
    );
    if (isSelected) {
      setSelectedCompanies((prev) =>
        prev.filter((c) => c.ticker !== company.ticker)
      );
      removeStockInfo(company.ticker);
    } else if (selectedCompanies.length < 3) {
      setSelectedCompanies((prev) => [...prev, company]);
      fetchStockInfo(company.ticker);
    } else {
      alert("You can only select up to 3 companies.");
    }
  };

  const handleGetRecommendation = () => {
    const prompt = generateRecommendationPrompt(
      selectedCompanies,
      financialData
    );
    fetchGPTRecommendation(prompt);
  };

  return (
    <>
      <Header />
      <Info open={infoOpen} setOpen={setInfoOpen} />
      <main className="flex min-h-screen flex-col bg-gray-900">
        <div className="flex-grow p-6 pt-24">
          <p className="mb-4 text-center text-sm font-bold text-red-500">
            THIS IS NOT FINANCIAL ADVICE. FOR EDUCATIONAL PURPOSES ONLY.
          </p>
          <h1 className="mb-6 text-center text-2xl font-bold text-green-500">
            Select Up to 3 {search} Companies to Compare
          </h1>

          <div className="relative isolate">
            {/* Background Gradient */}
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

            <div className="mx-auto flex max-w-7xl flex-col gap-8 md:flex-row">
              <CompanySelector
                techSectorData={techSectorData}
                selectedCompanies={selectedCompanies}
                onCompanyClick={handleCompanyClick}
              />
              <Dashboard
                selectedCompanies={selectedCompanies}
                financialData={financialData}
                esgData={esgData}
                tickers={tickers}
                setInfoOpen={setInfoOpen}
                recommendationProps={{
                  onGenerate: handleGetRecommendation,
                  recommendation,
                  loadingGPT,
                  recommendationRef,
                }}
              />
            </div>
            {error && (
              <p className="mt-4 text-center text-red-500">
                An error occurred fetching financial data. Please try again.
              </p>
            )}
          </div>
        </div>
      </main>
    </>
  );
}
