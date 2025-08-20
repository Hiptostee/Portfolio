"use client";

import { useState, useRef } from "react";
import { calculateIndex } from "../utils/calculations";

export function useGptRecommendation() {
  const [recommendation, setRecommendation] = useState("");
  const [loadingGPT, setLoadingGPT] = useState(false);
  const recommendationRef = useRef(null);

  const generateRecommendationPrompt = (selectedCompanies, financialData) => {
    const esgScale = `
**ESG Score Interpretation:**
- **0-10:** Negligible
- **10-20:** Low
- **20-40:** High
- **Above 40:** Severe

*Note: In this context, lower ESG scores are better.*
    `;

    const financialRatiosForIndex = {};
    Object.keys(financialData).forEach((ticker) => {
      financialRatiosForIndex[ticker] = {
        "Net Profit Margin (in %)": financialData[ticker].netProfitMargin,
      };
    });

    const companyDataStrings = selectedCompanies
      .map((company) => {
        const data = financialData[company.ticker];
        const financialMetrics = data
          ? `
- Revenue Growth: ${data.revenueGrowth || "-"}%
- Profit Margin: ${data.netProfitMargin?.toFixed(2) || "-"}%
- P/E Ratio: ${data.peRatio?.toFixed(2) || "-"}
- Dividend Yield: ${data.dividendYield?.toFixed(2) || "-"}%
                    `
          : `
- Revenue Growth: -
- Profit Margin: -
- P/E Ratio: -
- Dividend Yield: -
                    `;

        return `
${company.name} (${company.ticker}):
- Environmental Score: ${company.e || "-"}
- Social Score: ${company.s || "-"}
- Governance Score: ${company.g || "-"}
- Total ESG Score: ${company.esg || "-"}
- I4B Index: ${calculateIndex(company, financialRatiosForIndex).toFixed(2)}
${financialMetrics}
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

  const fetchGPTRecommendation = async (prompt) => {
    setLoadingGPT(true);
    setRecommendation("");
    try {
      const response = await fetch("/api/recommendations", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ prompt }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(
          errorData.error || "Failed to get recommendation from server."
        );
      }

      const data = await response.json();
      const content = data.recommendation || "No recommendation available.";

      setRecommendation(content);
      setTimeout(() => {
        recommendationRef.current?.scrollIntoView({
          behavior: "smooth",
          block: "start",
        });
      }, 100);
    } catch (error) {
      console.error("Error fetching recommendation: ", error);
      setRecommendation(`An error occurred: ${error.message}`);
    } finally {
      setLoadingGPT(false);
    }
  };

  return {
    recommendation,
    loadingGPT,
    fetchGPTRecommendation,
    generateRecommendationPrompt,
    recommendationRef,
    setRecommendation,
  };
}
