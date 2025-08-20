import ESGBarChart from "./esgbarchart";
import FinancialDataTable from "./financialDataTable";
import RecommendationEngine from "./recommendationEngine";

export default function Dashboard({
  selectedCompanies,
  financialData,
  esgData,
  tickers,
  setInfoOpen,
  recommendationProps,
}) {
  return (
    <div className="flex w-full flex-col gap-6 md:w-1/2">
      <RecommendationEngine
        onGenerate={recommendationProps.onGenerate}
        loadingGPT={recommendationProps.loadingGPT}
        recommendation={recommendationProps.recommendation}
        recommendationRef={recommendationProps.recommendationRef}
        disabled={
          selectedCompanies.length === 0 || recommendationProps.loadingGPT
        }
      />

      <div className="flex flex-col items-start">
        <h2 className="mb-2 text-xl font-bold text-green-600">ESG Scores</h2>
        <div className="w-full">
          <ESGBarChart data={esgData} ticker={tickers} />
        </div>
      </div>

      {selectedCompanies.length > 0 && (
        <FinancialDataTable
          selectedCompanies={selectedCompanies}
          financialData={financialData}
          setInfoOpen={setInfoOpen}
        />
      )}
    </div>
  );
}
