"use client";

import { useState } from "react";

export function useStockData() {
  const [financialData, setFinancialData] = useState({});
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(false);

  const fetchStockInfo = async (ticker) => {
    setLoading(true);
    setError(false);
    try {
      const response = await fetch(`/api/financial-data?ticker=${ticker}`);
      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.error || "Failed to fetch data.");
      }
      const data = await response.json();

      setFinancialData((prev) => ({
        ...prev,
        [ticker]: {
          companyName: data.profile.companyName,
          totalRevenue: data.incomeStatement?.[0]?.revenue,
          netIncome: data.incomeStatement?.[0]?.netIncome,
          currentRatio: data.ratios?.currentRatio || 0,
          peRatio: data.ratios?.priceEarningsRatio || 0,
          netProfitMargin: (data.ratios?.netProfitMargin || 0) * 100,
          dividendYield: (data.ratios?.dividendYield || 0) * 100,
          currentPrice: data.stockPrice?.price,
          yearHigh: data.stockPrice?.yearHigh,
          yearLow: data.stockPrice?.yearLow,
        },
      }));
    } catch (err) {
      console.error(`Error fetching stock info for ${ticker}: `, err);
      setError(true);
    } finally {
      setLoading(false);
    }
  };

  const removeStockInfo = (ticker) => {
    setFinancialData((prev) => {
      const { [ticker]: _, ...rest } = prev;
      return rest;
    });
  };

  return { financialData, loading, error, fetchStockInfo, removeStockInfo };
}
